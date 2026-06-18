"""tool_action_server.py
────────────────────────
영구 모션 액션 서버 — PlaceAtStaging(fetch) / ReturnToSlot(return) 액션 +
home / estop / estop_reset 서비스.

TCP GripperDA_v1 Z+160 상주 설정. (이전 데모 실패 원인 = TCP 미적용)
/robot/status (RobotStatus) 단일 발행자 → S-7 명령차단 복구.
E-stop: move_stop + 래치 (servo-off 미적용, S-3).

설계 원칙:
  - MultiThreadedExecutor + ReentrantCallbackGroup (일반) /
    MutuallyExclusiveCallbackGroup (E-stop, 고우선 독립)
  - _movel/_movej/_grip 은 threading.Event 패턴 사용
    (spin_until_future_complete 로 executor 데드락 방지)
  - E-stop 래치: set 이후 모든 신규 액션/home 즉시 거부
  - unit_actions/toolbox_motion 은 ROS2 의존 없음 — sys.path 로 직접 import
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from dsr_msgs2.srv import ConfigCreateTcp, MoveLine, MoveJoint, MoveStop, SetCurrentTcp
from interfaces.action import PlaceAtStaging, ReturnToSlot
from interfaces.msg import RobotStatus
from interfaces.srv import GripperSetPosition, LogEvent
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _add_unit_actions_to_path() -> None:
    """레포 루트에서 unit_actions/ 를 sys.path에 추가한다 (toolbox_seq_runner 동일 패턴)."""
    candidates: list[str] = []
    env_root = os.environ.get("FINAL_PROJECT_ROOT")
    if env_root:
        candidates.append(env_root)
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidates.append(here)
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for root in candidates:
        if os.path.isdir(os.path.join(root, "unit_actions")):
            if root not in sys.path:
                sys.path.insert(0, root)
            return
    raise RuntimeError(
        "unit_actions 경로를 찾을 수 없습니다. "
        "FINAL_PROJECT_ROOT 환경변수로 레포 루트를 지정하세요."
    )


_add_unit_actions_to_path()

from unit_actions.toolbox_motion import (  # noqa: E402
    ACC_L,
    ACC_R,
    PULSE_RELEASE,
    StepKind,
    VEL_L,
    VEL_R,
    drawer_close_seq,
    drawer_open_seq,
    full_socket_fetch_seq,
    full_socket_return_seq,
    home_seq,
)

# 그리퍼 파지력 (TaskWriter SubRoutine 실측) — pulse > PULSE_RELEASE 일 때만 인가
_GRIP_CURRENT: int = 400

DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

_QOS_ROBOT_STATUS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_TCP_NAME = "GripperDA_v1"
_TCP_POS = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]

# socket_19mm 보관 서랍 (full_socket_fetch_seq/full_socket_return_seq 와 동일 layer)
_TOOLBOX_LAYER = 1


class ToolActionServer(Node):
    """PlaceAtStaging(fetch) / ReturnToSlot(return) 영구 액션 서버."""

    def __init__(self) -> None:
        super().__init__("tool_action_server")

        self.declare_parameter("robot_ns", "dsr01")
        ns = self.get_parameter("robot_ns").get_parameter_value().string_value
        p = f"/{ns}"

        # ── 콜백 그룹 분리 ─────────────────────────────────────────────────
        self._normal_cbg = ReentrantCallbackGroup()        # 액션·home
        self._estop_cbg = MutuallyExclusiveCallbackGroup()  # E-stop (독립)

        # ── DSR 서비스 클라이언트 ──────────────────────────────────────────
        self._movel_cli = self.create_client(
            MoveLine, f"{p}/motion/move_line", callback_group=self._normal_cbg
        )
        self._movej_cli = self.create_client(
            MoveJoint, f"{p}/motion/move_joint", callback_group=self._normal_cbg
        )
        self._stop_cli = self.create_client(
            MoveStop, f"{p}/motion/move_stop", callback_group=self._estop_cbg
        )
        self._create_tcp_cli = self.create_client(
            ConfigCreateTcp, f"{p}/tcp/config_create_tcp", callback_group=self._normal_cbg
        )
        self._set_tcp_cli = self.create_client(
            SetCurrentTcp, f"{p}/tcp/set_current_tcp", callback_group=self._normal_cbg
        )
        self._gripper_cli = self.create_client(
            GripperSetPosition, "/gripper/set_position", callback_group=self._normal_cbg
        )
        self._log_event_cli = self.create_client(
            LogEvent, "/db/LogEvent", callback_group=self._normal_cbg
        )

        # ── 필수 서비스 대기 (시작 시 연결 확인) ──────────────────────────
        for cli, name in [
            (self._movel_cli, "move_line"),
            (self._movej_cli, "move_joint"),
            (self._gripper_cli, "gripper/set_position"),
        ]:
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"[TAS] {name} 없음 — bringup 먼저 실행")
                raise RuntimeError(f"{name} 서비스 없음")
            self.get_logger().info(f"[TAS] {name} 연결됨")

        # ── /robot/status 발행자 ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            RobotStatus, "/robot/status", _QOS_ROBOT_STATUS
        )
        # ── /plc/system_state 발행자 ──────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── 상태 ──────────────────────────────────────────────────────────
        self._estop_latch = threading.Event()  # E-stop 래치 (set 후 reset 전까지 모든 동작 거부)
        self._action_lock = threading.Lock()  # 동시 액션 방지

        # ── 액션 서버 ─────────────────────────────────────────────────────
        self._place_server = ActionServer(
            self,
            PlaceAtStaging,
            "place_at_staging",
            execute_callback=self._on_place_at_staging,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )
        self._return_server = ActionServer(
            self,
            ReturnToSlot,
            "return_to_slot",
            execute_callback=self._on_return_to_slot,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )

        # ── 서비스 서버 ───────────────────────────────────────────────────
        self._home_srv = self.create_service(
            Trigger, "~/home", self._on_home, callback_group=self._normal_cbg
        )
        self._estop_srv = self.create_service(
            Trigger, "~/estop", self._on_estop, callback_group=self._estop_cbg
        )
        self._estop_reset_srv = self.create_service(
            Trigger, "~/estop_reset", self._on_estop_reset, callback_group=self._estop_cbg
        )
        self._open_toolbox_srv = self.create_service(
            Trigger, "~/open_toolbox", self._on_open_toolbox, callback_group=self._normal_cbg
        )
        self._close_toolbox_srv = self.create_service(
            Trigger, "~/close_toolbox", self._on_close_toolbox, callback_group=self._normal_cbg
        )

        # ── TCP 초기 설정 타이머 ──────────────────────────────────────────
        self._tcp_timer = self.create_timer(
            2.0, self._tcp_setup_once, callback_group=self._normal_cbg
        )

        self._publish_status(is_moving=False)
        self.get_logger().info("[TAS] tool_action_server 준비 완료")

    # ── TCP 설정 ────────────────────────────────────────────────────────────

    def _tcp_setup_once(self) -> None:
        self._tcp_timer.cancel()
        self._set_tcp()

    def _set_tcp(self) -> None:
        """TCP 등록 + 활성화 (fire-and-forget, chamjo 동일 패턴)."""
        try:
            if self._create_tcp_cli.service_is_ready():
                req = ConfigCreateTcp.Request()
                req.name = _TCP_NAME
                req.pos = _TCP_POS
                self._create_tcp_cli.call_async(req)
                self.get_logger().info(f"[TAS] TCP 등록 요청: {_TCP_NAME}")
            else:
                self.get_logger().warn("[TAS] config_create_tcp 미준비 — 건너뜀")
            if self._set_tcp_cli.service_is_ready():
                req2 = SetCurrentTcp.Request()
                req2.name = _TCP_NAME
                self._set_tcp_cli.call_async(req2)
                self.get_logger().info(f"[TAS] TCP 활성화 요청: {_TCP_NAME}")
            else:
                self.get_logger().warn("[TAS] set_current_tcp 미준비 — 건너뜀")
            time.sleep(0.3)
        except Exception as exc:
            self.get_logger().warn(f"[TAS] TCP 설정 예외 (무시): {exc}")

    # ── /robot/status 발행 ──────────────────────────────────────────────────

    def _publish_status(self, is_moving: bool) -> None:
        msg = RobotStatus()
        msg.is_moving = is_moving
        self._status_pub.publish(msg)

    def _set_plc(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._plc_pub.publish(msg)

    # ── 액션 goal/cancel 핸들러 ─────────────────────────────────────────────

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._estop_latch.is_set():
            self.get_logger().warn("[TAS] goal 거부 — E-stop 래치 활성")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ── 액션 실행 ───────────────────────────────────────────────────────────

    def _on_place_at_staging(self, goal_handle) -> PlaceAtStaging.Result:
        """fetch: 서랍 열기 → 소켓 파지 → 스테이징 거치 → 서랍 닫기."""
        result = PlaceAtStaging.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(f"[TAS] fetch 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")

            steps = full_socket_fetch_seq()
            ok = self._run_sequence(steps, goal_handle, action_class=PlaceAtStaging)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"fetch 완료: {tool_id}"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] fetch 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"fetch 실패: {tool_id}"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] fetch 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_plc("error")
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()

        return result

    def _on_return_to_slot(self, goal_handle) -> ReturnToSlot.Result:
        """return: 서랍 열기 → 스테이징 픽업 → 슬롯 반납 → 서랍 닫기."""
        result = ReturnToSlot.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(f"[TAS] return 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")

            steps = full_socket_return_seq()
            ok = self._run_sequence(steps, goal_handle, action_class=ReturnToSlot)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"return 완료: {tool_id}"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] return 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"return 실패: {tool_id}"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] return 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_plc("error")
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()

        return result

    # ── 서비스 핸들러 ───────────────────────────────────────────────────────

    def _on_home(self, _req, response: Trigger.Response) -> Trigger.Response:
        if self._estop_latch.is_set():
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info("[TAS] home 시작")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            ok = self._run_sequence(home_seq())
            if ok:
                response.success = True
                response.message = "home 완료"
                self._set_plc("idle")
            else:
                response.success = False
                response.message = "home 실패"
                self._set_plc("error")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()
        return response

    def _on_open_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        """공구함 서랍 열기 (단독 sub-task — fetch/return 없이 서랍만 개방).

        전제조건: 서랍이 닫힌 상태(LAYER{n}_APPROACH 진입 가능)여야 한다.
        이미 열린 서랍에 호출하면 GRIP_BOX 단계에서 충돌 위험 — 운영자가
        대시보드에서 서랍 상태를 육안 확인 후 호출할 것.
        """
        return self._run_toolbox_seq(
            seq=drawer_open_seq(_TOOLBOX_LAYER),
            label="open_toolbox",
            response=response,
        )

    def _on_close_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        """공구함 서랍 닫기 (단독 sub-task — fetch/return 없이 서랍만 폐쇄).

        전제조건: 서랍이 열린 상태(팔이 LAYER{n}_INNER 부근)여야 한다.
        이미 닫힌 서랍에 호출하면 충돌 위험 — 운영자가 대시보드에서
        서랍 상태를 육안 확인 후 호출할 것.
        """
        return self._run_toolbox_seq(
            seq=drawer_close_seq(_TOOLBOX_LAYER),
            label="close_toolbox",
            response=response,
        )

    def _run_toolbox_seq(
        self, seq: list, label: str, response: Trigger.Response
    ) -> Trigger.Response:
        if self._estop_latch.is_set():
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info(f"[TAS] {label} 시작")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            ok = self._run_sequence(seq)
            if ok:
                response.success = True
                response.message = f"{label} 완료"
                self._set_plc("idle")
            else:
                response.success = False
                response.message = f"{label} 실패"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] {label} 예외: {exc}")
            response.success = False
            response.message = str(exc)
            self._set_plc("error")
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()
        return response

    def _on_estop(self, _req, response: Trigger.Response) -> Trigger.Response:
        """E-stop: move_stop + 래치. 고우선 콜백 그룹(estop_cbg)에서 실행."""
        self.get_logger().error("[TAS] E-STOP 요청 수신")
        self._estop_latch.set()
        self._publish_status(is_moving=False)
        self._set_plc("e_stop")

        # DSR 즉시 정지
        try:
            if self._stop_cli.service_is_ready():
                req = MoveStop.Request()
                req.stop_type = 0  # 감속 정지
                self._stop_cli.call_async(req)
                self.get_logger().error("[TAS] move_stop 요청 완료")
            else:
                self.get_logger().error("[TAS] move_stop 서비스 미준비")
        except Exception as exc:
            self.get_logger().error(f"[TAS] move_stop 예외: {exc}")

        # DB 로그 (best-effort)
        self._log_event_async("e_stop", "E-stop 버튼 트리거")

        response.success = True
        response.message = "E-stop 래치 활성 — estop_reset 으로만 해제"
        return response

    def _on_estop_reset(self, _req, response: Trigger.Response) -> Trigger.Response:
        """E-stop 래치 해제 — 운영자 명시적 확인 후만 허용."""
        self.get_logger().warn("[TAS] E-stop 래치 해제")
        self._estop_latch.clear()
        self._set_plc("idle")
        self._log_event_async("e_stop_reset", "운영자 E-stop 래치 해제")
        response.success = True
        response.message = "E-stop 래치 해제 완료"
        return response

    # ── 시퀀스 실행 ─────────────────────────────────────────────────────────

    def _run_sequence(
        self, steps: list, goal_handle=None, action_class=None
    ) -> bool:
        total = len(steps)
        for i, step in enumerate(steps):
            if self._estop_latch.is_set():
                self.get_logger().warn(f"[TAS] E-stop 래치 — step {i+1} 중단")
                return False
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False

            self.get_logger().info(f"  step {i+1}/{total}: {step.kind.name}")

            if goal_handle is not None and action_class is not None:
                fb = action_class.Feedback()
                fb.phase = step.kind.name
                fb.progress = float(i) / float(total)
                goal_handle.publish_feedback(fb)

            ok = self._exec_step(step)
            if not ok:
                self.get_logger().error(f"  step {i+1} 실패 — 중단")
                return False

            # step.marker("pick"/"place")가 설정된 step은 실행 직후 추가
            # feedback을 발행한다 — orchestrator가 이를 받아 DB 상태를
            # 물리적 집기/놓기 시점에 맞춰 전이시킨다.
            if goal_handle is not None and action_class is not None and step.marker:
                fb = action_class.Feedback()
                fb.phase = step.marker
                fb.progress = float(i + 1) / float(total)
                goal_handle.publish_feedback(fb)
        return True

    def _exec_step(self, step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return self._movel(step, DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return self._movel(step, DR_MV_MOD_REL)
        elif step.kind in (StepKind.MOVE_J_ABS, StepKind.MOVE_J_REL):
            return self._movej(step)
        elif step.kind == StepKind.GRIP:
            return self._grip(step)
        elif step.kind == StepKind.WAIT:
            time.sleep(step.sec or 0.5)
            return True
        self.get_logger().warn(f"  알 수 없는 StepKind: {step.kind}")
        return False

    # ── 모션 헬퍼 (threading.Event 패턴 — executor 데드락 방지) ──────────────

    def _wait_future(self, fut: Any, timeout: float) -> bool:
        """future 완료를 blocking 없이 대기. spin_until_future_complete 사용 금지."""
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        return done.wait(timeout=timeout)

    def _movel(self, step, mode: int) -> bool:
        req = MoveLine.Request()
        req.pos = [float(v) for v in step.pose]
        # MoveLine vel/acc는 [병진, 회전] 2원소. Step은 병진 스칼라만 보유하므로
        # 회전 성분은 TaskWriter 실측 상수(VEL_R/ACC_R)로 고정한다.
        req.vel = [step.vel or VEL_L, VEL_R]
        req.acc = [step.acc or ACC_L, ACC_R]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = mode
        req.blend_type = 0
        req.sync_type = 0
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=30.0):
            self.get_logger().error(f"  move_line 타임아웃: pos={step.pose}")
            return False
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"  move_line 실패: pos={step.pose}")
        return ok

    def _movej(self, step) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(v) for v in step.pose]
        req.vel = step.vel or 12.0
        req.acc = step.acc or 20.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0
        fut = self._movej_cli.call_async(req)
        if not self._wait_future(fut, timeout=20.0):
            self.get_logger().error(f"  move_joint 타임아웃: pos={step.pose}")
            return False
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"  move_joint 실패: pos={step.pose}")
        return ok

    def _grip(self, step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0
        current = _GRIP_CURRENT if pulse > PULSE_RELEASE else 0
        req = GripperSetPosition.Request()
        req.position = pulse
        req.current = current
        req.timeout_sec = 0.0
        fut = self._gripper_cli.call_async(req)
        if not self._wait_future(fut, timeout=5.0):
            self.get_logger().error("  gripper 타임아웃")
            return False
        res = fut.result()
        if res is None or not res.success:
            msg = res.message if res else "timeout"
            self.get_logger().error(f"  gripper 실패: {msg}")
            return False
        time.sleep(0.1)
        return True

    def _log_event_async(self, event_type: str, notes: str) -> None:
        try:
            if not self._log_event_cli.service_is_ready():
                return
            req = LogEvent.Request()
            req.tool_id = ""
            req.event_type = event_type
            req.track = "A"
            req.notes = notes
            self._log_event_cli.call_async(req)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ToolActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except RuntimeError as e:
        print(f"[TAS] 초기화 실패: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
