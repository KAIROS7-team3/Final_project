"""tool_action_server.py
────────────────────────
ExecutePhase 영구 모션 액션 서버 (Track A BT 오케스트레이션 대상).

단일 ExecutePhase 액션으로 PlaceAtStaging / ReturnToSlot 을 대체한다.
phase 문자열로 시퀀스 선택 → SequenceEngine 이 DSR 서비스 실행.

phase 매핑:
  open_drawer  → drawer_open_seq(layer_id)
  fetch        → fixed_fetch_seq()       (비전-free, toolbox.yaml 고정좌표)
  return       → vision_return_seq()     (그리퍼 캠 비전 기반 staging 픽업)
  close_drawer → drawer_close_seq(layer_id)
  home         → home_seq()

서비스:
  ~/home          — 홈 복귀 (E-stop 래치·동작 중 거부)
  ~/estop         — DSR 즉시 정지 + 래치 (S-3)
  ~/estop_reset   — 래치 해제 (운영자 명시 확인 후)
  ~/open_toolbox  — 서랍 열기 단독 (layer 1 고정)
  ~/close_toolbox — 서랍 닫기 단독 (layer 1 고정)

설계:
  - SequenceEngine 이 DSR 클라이언트·비전 구독·E-stop 감시를 모두 소유
  - 액션 서버는 phase 매핑 + goal 수명 관리 담당. /robot/status 발행은 orchestrator 소유 (SetMoving BT 노드).
  - E-stop: engine.request_stop() → engine.estop_triggered 래치 → goal 거부
  - ~/estop_reset: engine.reset_estop() 호출 (운영자 명시 확인 후)
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from interfaces.action import ExecutePhase
from motion.sequence_engine import SequenceEngine

from unit_actions.toolbox_motion import (
    drawer_close_seq,
    drawer_open_seq,
    drawer_open_seq_v2,
    drawer_close_seq_v2,
    scan_layer_seq,
    fixed_fetch_seq,
    home_seq,
    vision_return_seq,
    stage_pick_test_seq,
)

_VALID_PHASES = frozenset({
    "open_drawer", "fetch", "return", "close_drawer", "home",
    "open_drawer_scan", "close_drawer_scan",   # 스캔 전용 v2 서랍 시퀀스
    "scan_pose", "scan_pose_fetch", "scan_pose_return",
    "stage_pick_test",
})
_DEFAULT_TCP_NAME   = "GripperDA_v1"
_DEFAULT_CONFIG_PATH = "/home/kg/assistant/config/toolbox.yaml"
_STANDALONE_LAYER   = 1   # ~/open_toolbox · ~/close_toolbox 기본 layer


class ExecutePhaseServer(Node):
    """ExecutePhase 영구 액션 서버 — SequenceEngine 위임."""

    def __init__(self) -> None:
        super().__init__("tool_action_server")  # 노드명 유지 (launch 호환)

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter("robot_ns",    "dsr01")
        self.declare_parameter("tcp_name",    _DEFAULT_TCP_NAME)
        self.declare_parameter("config_path", _DEFAULT_CONFIG_PATH)
        self.declare_parameter("mode",        "virtual")

        ns   = self.get_parameter("robot_ns").get_parameter_value().string_value
        tcp  = self.get_parameter("tcp_name").get_parameter_value().string_value
        cfg  = self.get_parameter("config_path").get_parameter_value().string_value
        mode = self.get_parameter("mode").get_parameter_value().string_value

        # ── 콜백 그룹 분리 ──────────────────────────────────────────────────
        self._normal_cbg = ReentrantCallbackGroup()
        self._estop_cbg  = MutuallyExclusiveCallbackGroup()

        # ── 동시 goal 방지 락 ────────────────────────────────────────────────
        self._action_lock = threading.Lock()

        # ── SequenceEngine 초기화 ────────────────────────────────────────────
        self._engine = SequenceEngine(
            self,
            robot_ns=ns,
            tcp_name=tcp,
            config_path=cfg,
            mode=mode,
        )
        self._engine.setup(wait_timeout_sec=15.0)

        # ── 발행자 ──────────────────────────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── ExecutePhase 액션 서버 ───────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            ExecutePhase,
            "execute_phase",
            execute_callback=self._on_execute_phase,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )

        # ── 서비스 서버 ─────────────────────────────────────────────────────
        self._home_srv = self.create_service(
            Trigger, "~/home", self._on_home, callback_group=self._normal_cbg
        )
        self._estop_srv = self.create_service(
            Trigger, "~/estop", self._on_estop, callback_group=self._estop_cbg
        )
        self._estop_reset_srv = self.create_service(
            Trigger, "~/estop_reset", self._on_estop_reset, callback_group=self._estop_cbg
        )
        self._open_srv = self.create_service(
            Trigger, "~/open_toolbox", self._on_open_toolbox, callback_group=self._normal_cbg
        )
        self._close_srv = self.create_service(
            Trigger, "~/close_toolbox", self._on_close_toolbox, callback_group=self._normal_cbg
        )
        self._open_srv_l2 = self.create_service(
            Trigger, "~/open_toolbox_l2", self._on_open_toolbox_l2, callback_group=self._normal_cbg
        )
        self._close_srv_l2 = self.create_service(
            Trigger, "~/close_toolbox_l2", self._on_close_toolbox_l2, callback_group=self._normal_cbg
        )

        # ── TCP 초기 설정 타이머 ─────────────────────────────────────────────
        self._tcp_timer = self.create_timer(
            2.0, self._tcp_setup_once, callback_group=self._normal_cbg
        )

        self.get_logger().info("[TAS] ExecutePhase 서버 준비 완료")

    # ── TCP 설정 ─────────────────────────────────────────────────────────────

    def _tcp_setup_once(self) -> None:
        self._tcp_timer.cancel()
        self._engine.set_tcp()

    def _set_plc(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._plc_pub.publish(msg)

    # ── 액션 goal / cancel 핸들러 ────────────────────────────────────────────

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._engine.estop_triggered:
            self.get_logger().warn("[TAS] goal 거부 — E-stop 래치 활성")
            return GoalResponse.REJECT
        phase = getattr(goal_request, "phase", "")
        if phase not in _VALID_PHASES:
            self.get_logger().warn(f"[TAS] goal 거부 — 알 수 없는 phase: {phase!r}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ── ExecutePhase 실행 ────────────────────────────────────────────────────

    def _on_execute_phase(self, goal_handle) -> ExecutePhase.Result:
        result  = ExecutePhase.Result()
        req     = goal_handle.request
        phase   = req.phase
        tool_id = req.tool_id
        layer_id = req.layer_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(
                f"[TAS] phase={phase} tool_id={tool_id!r} layer_id={layer_id}"
            )
            self._engine.set_tcp()
            self._set_plc("moving")

            steps = self._resolve_phase(phase, tool_id, layer_id)
            if steps is None:
                goal_handle.abort()
                result.success = False
                result.message = f"알 수 없는 phase: {phase!r}"
                return result

            def feedback_cb(step_name: str, progress: float) -> None:
                fb = ExecutePhase.Feedback()
                fb.phase    = step_name
                fb.progress = progress
                goal_handle.publish_feedback(fb)

            ok = self._engine.run_sequence(
                steps,
                tool_id=tool_id,
                cancel_check=lambda: goal_handle.is_cancel_requested,
                feedback_cb=feedback_cb,
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            elif ok:
                goal_handle.succeed()
                result.success = True
                result.message = f"{phase} 완료"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] {phase} 완료: tool_id={tool_id!r}")
            else:
                self._engine.stop_motion()
                goal_handle.abort()
                result.success = False
                result.message = f"{phase} 실패"
                self._set_plc("error")
                self.get_logger().error(f"[TAS] {phase} 실패: tool_id={tool_id!r}")

        except Exception as exc:
            self.get_logger().error(f"[TAS] phase={phase} 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_plc("error")
        finally:
            self._action_lock.release()

        return result

    # ── phase → 시퀀스 매핑 ──────────────────────────────────────────────────

    def _resolve_phase(
        self, phase: str, tool_id: str, layer_id: int
    ) -> Optional[list]:
        if phase == "open_drawer":
            return drawer_open_seq_v2(layer_id)
        if phase == "fetch":
            return fixed_fetch_seq()
        if phase == "return":
            return vision_return_seq(scan_j_deg=self._engine._return_scan_j_deg)
        if phase == "close_drawer":
            return drawer_close_seq_v2(layer_id)
        if phase == "home":
            return home_seq()
        if phase == "open_drawer_scan":
            return drawer_open_seq_v2(layer_id)
        if phase == "close_drawer_scan":
            return drawer_close_seq_v2(layer_id)
        if phase in ("scan_pose", "scan_pose_fetch"):
            self._engine._load_toolbox_config()
            return scan_layer_seq(getattr(self._engine, "_fetch_scan_j_deg", None))
        if phase == "scan_pose_return":
            self._engine._load_toolbox_config()
            return scan_layer_seq(getattr(self._engine, "_return_scan_j_deg", None))
        if phase == "stage_pick_test":
            self._engine._load_toolbox_config()
            return stage_pick_test_seq(scan_j_deg=getattr(self._engine, "_return_scan_j_deg", None))
        return None

    # ── 서비스 핸들러 ────────────────────────────────────────────────────────

    def _on_home(self, _req, response: Trigger.Response) -> Trigger.Response:
        # ⚠️ S-7: 이 서비스는 BT 외부(유지보수·긴급 복귀)에서만 호출할 것.
        # BT가 실행 중이 아닐 때만 안전하다 — 호출 중 /robot/status가 갱신되지 않으므로
        # voice 게이트가 열린 채로 로봇이 움직일 수 있다.
        if self._engine.estop_triggered:
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info("[TAS] home 시작")
            self._engine.set_tcp()
            self._set_plc("moving")
            ok = self._engine.run_sequence(home_seq())
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
            self._action_lock.release()
        return response

    def _on_open_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_open_seq_v2(_STANDALONE_LAYER), "open_toolbox_l1", response
        )

    def _on_close_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_close_seq_v2(_STANDALONE_LAYER), "close_toolbox_l1", response
        )

    def _on_open_toolbox_l2(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_open_seq_v2(1), "open_toolbox_l2", response
        )

    def _on_close_toolbox_l2(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_close_seq_v2(1), "close_toolbox_l2", response
        )

    def _run_service_seq(
        self, seq: list, label: str, response: Trigger.Response
    ) -> Trigger.Response:
        if self._engine.estop_triggered:
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info(f"[TAS] {label} 시작")
            self._engine.set_tcp()
            self._set_plc("moving")
            ok = self._engine.run_sequence(seq)
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
            self._action_lock.release()
        return response

    def _on_estop(self, _req, response: Trigger.Response) -> Trigger.Response:
        """S-3: E-stop — DSR 즉시 정지 + 래치. 고우선 콜백 그룹에서 실행."""
        self.get_logger().error("[TAS] E-STOP 요청 수신")
        self._engine.request_stop()
        self._set_plc("e_stop")
        response.success = True
        response.message = "E-stop 래치 활성 — estop_reset 으로만 해제"
        return response

    def _on_estop_reset(self, _req, response: Trigger.Response) -> Trigger.Response:
        """E-stop 래치 해제 — 운영자 명시적 확인 후만 허용."""
        self.get_logger().warn("[TAS] E-stop 래치 해제")
        self._engine.reset_estop()
        self._set_plc("idle")
        response.success = True
        response.message = "E-stop 래치 해제 완료"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExecutePhaseServer()
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
