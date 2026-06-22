"""toolbox_seq_runner.py
────────────────────────
toolbox_motion.py 시퀀스를 virtual/real 모드에서 실행하는 테스트 노드.

실행:
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=open_0
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_fetch \\
    -p tool_id:=screwdriver -p vision_x_mm:=300.0 ...

  sequence 옵션:
    open_0  / close_0  — layer 0 (1층 서랍)
    open_1  / close_1  — layer 1 (2층 서랍)
    socket_fetch / socket_return — 소켓 공구 (tool_id 필수)
    fixed_fetch          — 고정좌표 fetch (tool_id 필수, toolbox.yaml grasp_pose_base 사용)
    vision_fetch / vision_return — 비전 기반 공구 (tool_id 필수)

단위: toolbox_motion.py 좌표는 DSR 네이티브(mm/deg) → move_line/move_joint 직접 전달.

모션 실행 로직은 노드 비의존 `SequenceEngine`(sequence_engine.py)으로 분리되어 있다.
이 러너는 파라미터 파싱·DB gate·PLC 오류 표시·is_moving·시퀀스 선택 같은 정책을
담당하고, 실제 step 실행은 엔진에 위임한다.
"""

import logging
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from typing import Optional


def _add_unit_actions_to_path() -> None:
    """unit_actions/ 는 ros2_ws 밖(레포 루트)에 있고 ROS2 패키지가 아니라
    colcon이 설치하지 않으므로, 소스 트리의 레포 루트를 sys.path에 추가한다.
    db_core/, plc_core/도 같은 레포 루트에 있어 동일 경로로 커버된다.
    """
    candidates = []
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
        "unit_actions 경로를 찾을 수 없습니다. 레포 루트를 FINAL_PROJECT_ROOT "
        "환경변수로 지정하세요 (예: export FINAL_PROJECT_ROOT=~/Final_project)."
    )


_add_unit_actions_to_path()

from unit_actions.toolbox_motion import (
    VEL_L, ACC_L, VEL_R, ACC_R, VEL_J, ACC_J,
    home_seq,
    drawer_open_seq,
    drawer_close_seq,
    drawer_open_seq_v2,
    drawer_close_seq_v2,
    socket_fetch_seq,
    socket_return_seq,
    fixed_fetch_seq,
    vision_fetch_seq,
    vision_return_seq,
    vision_drawer_open_seq,
    vision_drawer_close_seq,
)
from db_core.client import DBClient, DBError, DBCacheExpiredError
from plc_core.client import PLCClient

from motion.sequence_engine import SequenceEngine

# S-7 / S-3: Transient Local QoS — 구독 시 최신 retained 값 즉시 수신
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# S-2: DB gate 적용 대상 시퀀스
# fetch/return: 공구 단위 gate (tool_id + check_feasibility)
# open/close:   서랍(layer) 단위 gate (layer_id + check_drawer_feasibility, tool_id 불필요)
_FETCH_SEQS  = {'vision_fetch', 'socket_fetch'}
_RETURN_SEQS = {'vision_return', 'socket_return'}
_GATE_SEQS   = _FETCH_SEQS | _RETURN_SEQS
_DRAWER_SEQS = {'vision_open_0', 'vision_open_1', 'vision_close_0', 'vision_close_1'}


class ToolboxSeqRunner(Node):

    def __init__(self) -> None:
        super().__init__('toolbox_seq_runner')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('sequence', 'open_0')
        self.declare_parameter('mode', 'virtual')
        self.declare_parameter('tcp_name', 'GripperDA_v1')
        self.declare_parameter('tool_id', '')        # S-2: fetch/return 시퀀스에 필수
        self.declare_parameter('db_path', 'robot_arm.db')
        self.declare_parameter('vision_x_mm', 0.0)
        self.declare_parameter('vision_y_mm', 0.0)
        self.declare_parameter('vision_z_mm', 0.0)
        self.declare_parameter('bottom_x_mm', 0.0)
        self.declare_parameter('bottom_y_mm', 0.0)
        self.declare_parameter('bottom_z_mm', 0.0)
        self.declare_parameter('slot_x_mm', 0.0)
        self.declare_parameter('slot_y_mm', 0.0)
        self.declare_parameter('slot_z_mm', 0.0)
        self.declare_parameter('approach_x_mm', 0.0)
        self.declare_parameter('approach_y_mm', 0.0)
        self.declare_parameter('approach_z_mm', 0.0)
        # S-5/E-4: 속도·가속도 상한 — config/robot_poses.yaml motion_limits 값으로 오버라이드 가능
        self.declare_parameter('vel_l', VEL_L)
        self.declare_parameter('acc_l', ACC_L)
        self.declare_parameter('vel_r', VEL_R)
        self.declare_parameter('acc_r', ACC_R)
        self.declare_parameter('vel_j', VEL_J)
        self.declare_parameter('acc_j', ACC_J)

        ns          = self.get_parameter('robot_ns').get_parameter_value().string_value
        seq_name    = self.get_parameter('sequence').get_parameter_value().string_value
        self._mode     = self.get_parameter('mode').get_parameter_value().string_value
        self._tcp_name = self.get_parameter('tcp_name').get_parameter_value().string_value
        self._tool_id  = self.get_parameter('tool_id').get_parameter_value().string_value
        db_path        = self.get_parameter('db_path').get_parameter_value().string_value
        self._vision_x = self.get_parameter('vision_x_mm').get_parameter_value().double_value
        self._vision_y = self.get_parameter('vision_y_mm').get_parameter_value().double_value
        self._vision_z = self.get_parameter('vision_z_mm').get_parameter_value().double_value
        self._bottom_x = self.get_parameter('bottom_x_mm').get_parameter_value().double_value
        self._bottom_y = self.get_parameter('bottom_y_mm').get_parameter_value().double_value
        self._bottom_z = self.get_parameter('bottom_z_mm').get_parameter_value().double_value
        self._slot_x     = self.get_parameter('slot_x_mm').get_parameter_value().double_value
        self._slot_y     = self.get_parameter('slot_y_mm').get_parameter_value().double_value
        self._slot_z     = self.get_parameter('slot_z_mm').get_parameter_value().double_value
        self._approach_x = self.get_parameter('approach_x_mm').get_parameter_value().double_value
        self._approach_y = self.get_parameter('approach_y_mm').get_parameter_value().double_value
        self._approach_z = self.get_parameter('approach_z_mm').get_parameter_value().double_value
        vel_l = self.get_parameter('vel_l').get_parameter_value().double_value
        acc_l = self.get_parameter('acc_l').get_parameter_value().double_value
        vel_r = self.get_parameter('vel_r').get_parameter_value().double_value
        acc_r = self.get_parameter('acc_r').get_parameter_value().double_value
        vel_j = self.get_parameter('vel_j').get_parameter_value().double_value
        acc_j = self.get_parameter('acc_j').get_parameter_value().double_value

        self._cb_group = ReentrantCallbackGroup()

        # S-7: 기동 전 is_moving 상태 — Transient Local 구독으로 다른 노드의 retained 값 수신
        self._prev_is_moving: bool = False

        # E-4: config/toolbox.yaml 경로 (vision_motion 섹션) — 엔진에 전달
        cfg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../../config/toolbox.yaml',
        )

        # 모션 실행 엔진 — DSR 서비스·비전 구독·E-stop·step 실행 소유 (DB/PLC 미포함)
        self._engine = SequenceEngine(
            self,
            robot_ns=ns,
            tcp_name=self._tcp_name,
            config_path=cfg_path,
            mode=self._mode,
            vel_l=vel_l, acc_l=acc_l,
            vel_r=vel_r, acc_r=acc_r,
            vel_j=vel_j, acc_j=acc_j,
        )
        # 클라이언트·구독 생성 + 필수 서비스 대기 (없으면 RuntimeError)
        self._engine.setup(wait_timeout_sec=10.0)

        # S-7: is_moving pub/sub — Transient Local로 retained 값 유지·수신
        # 구독을 먼저 생성해 다른 노드의 retained True를 _run_once 전에 수신한다.
        # True publish는 is_moving 확인 후 _run_once에서 수행한다.
        self._is_moving_pub = self.create_publisher(Bool, '/motion/is_moving', _LATCHED_QOS)
        self._is_moving_sub = self.create_subscription(
            Bool, '/motion/is_moving', self._on_is_moving, _LATCHED_QOS,
            callback_group=self._cb_group,
        )

        # S-2: DB 클라이언트 — fetch/return 실행 전 feasibility 판정
        self._db: DBClient | None = None
        try:
            self._db = DBClient(db_path)
            self._db.connect()
        except Exception as e:
            self.get_logger().warn(f'[runner] DB 연결 실패 (fetch/return 실행 불가): {e}')
            try:
                self._plc.set_error()  # F8: DB 미연결 시 PLC 빨강으로 운영자 즉시 인지
            except Exception:
                pass

        # E-5: PLC 클라이언트 — 시퀀스 실패 시 오류 상태 표시
        self._plc = PLCClient()
        self._plc.connect()

        self._seq_name = seq_name
        self._done = False
        # 0.5s 후 _run_once 실행 — 그 사이 is_moving 구독 콜백이 retained 값을 수신할 수 있음
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb_group)

    @property
    def _estop_triggered(self) -> bool:
        """S-3: E-stop 상태는 엔진이 /plc/e_stop 구독을 소유 — 엔진 플래그 참조."""
        return self._engine.estop_triggered

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _on_is_moving(self, msg: Bool) -> None:
        """S-7: is_moving 토픽 수신 — 기동 전 다른 시퀀스 실행 여부 확인용."""
        self._prev_is_moving = msg.data

    # ── 메인 실행 ─────────────────────────────────────────────────────────

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        # S-7: 다른 시퀀스가 is_moving=True를 발행 중이면 거부
        if self._prev_is_moving:
            self.get_logger().error('[runner] is_moving=True 수신 — 다른 시퀀스 실행 중, 거부')
            rclpy.shutdown()
            return

        seq = self._resolve_sequence(self._seq_name)
        if seq is None:
            self.get_logger().error(f'[runner] 알 수 없는 sequence: {self._seq_name}')
            self.get_logger().error(
                '[runner] 사용 가능: home open_0 close_0 open_1 close_1 '
                'open_0v2 close_0v2 open_1v2 close_1v2 '
                'fixed_fetch socket_fetch socket_return vision_fetch vision_return '
                'vision_open_0 vision_open_1 vision_close_0 vision_close_1'
            )
            rclpy.shutdown()
            return

        # S-2: DB gate — fetch/return 시퀀스만 적용
        if self._seq_name in _GATE_SEQS:
            if not self._tool_id:
                self.get_logger().error('[runner] fetch/return 시퀀스에 tool_id 파라미터 필수 (S-2)')
                rclpy.shutdown()
                return
            if self._db is None:
                self.get_logger().error('[runner] DB 연결 없음 — fetch/return 실행 불가 (S-2)')
                self._plc.set_error()
                rclpy.shutdown()
                return
            intent = 'fetch' if self._seq_name in _FETCH_SEQS else 'return'
            try:
                feasible, reason = self._db.check_feasibility(intent, self._tool_id)
            except DBCacheExpiredError as e:
                self.get_logger().error(f'[runner] DB 캐시 만료 — 명령 거부 (S-2): {e}')
                self._plc.set_error()
                rclpy.shutdown()
                return
            except DBError as e:
                self.get_logger().error(f'[runner] DB 오류 — 명령 거부 (S-2): {e}')
                self._plc.set_error()
                rclpy.shutdown()
                return
            if not feasible:
                self.get_logger().error(
                    f'[runner] DB gate 차단 — tool_id={self._tool_id} reason={reason}'
                )
                self._plc.set_error()
                rclpy.shutdown()
                return

        # S-2: DB gate — open/close 서랍 단위 gate (tool_id 불필요, layer_id로 검사)
        if self._seq_name in _DRAWER_SEQS:
            if self._db is None:
                self.get_logger().error('[runner] DB 연결 없음 — open/close 실행 불가 (S-2)')
                self._plc.set_error()
                rclpy.shutdown()
                return
            intent   = 'open' if 'open' in self._seq_name else 'close'
            layer_id = int(self._seq_name[-1])
            try:
                feasible, reason = self._db.check_drawer_feasibility(intent, layer_id)
            except DBError as e:
                self.get_logger().error(f'[runner] DB 오류 — open/close 거부 (S-2): {e}')
                self._plc.set_error()
                rclpy.shutdown()
                return
            if not feasible:
                if reason == 'already_open':
                    # 이미 열린 서랍 — 오류 아님, 시퀀스 생략 후 정상 종료
                    self.get_logger().info(
                        f'[runner] 서랍 layer={layer_id} 이미 열림 — open 시퀀스 생략'
                    )
                    self._is_moving_pub.publish(Bool(data=False))
                    rclpy.shutdown()
                    return
                self.get_logger().error(
                    f'[runner] DB gate 차단 — intent={intent} layer={layer_id} reason={reason}'
                )
                self._plc.set_error()
                rclpy.shutdown()
                return

        # S-7: 시퀀스 시작 직전 is_moving=True 발행
        self._is_moving_pub.publish(Bool(data=True))

        if not self._engine.set_tcp():
            self.get_logger().error('[runner] TCP 설정 실패 — 시퀀스 중단')
            self._on_sequence_failure()
            self._is_moving_pub.publish(Bool(data=False))
            rclpy.shutdown()
            return

        self.get_logger().info(f'[runner] 시퀀스 시작: {self._seq_name} ({len(seq)} steps)')
        ok = self._engine.run_sequence(seq, tool_id=self._tool_id)

        if ok:
            self.get_logger().info(f'[runner] 시퀀스 완료: {self._seq_name}')
            if self._seq_name in _DRAWER_SEQS and self._db is not None:
                try:
                    intent   = 'open' if 'open' in self._seq_name else 'close'
                    layer_id = int(self._seq_name[-1])
                    self._db.update_drawer_state(layer_id, intent)
                except Exception as e:
                    self.get_logger().error(
                        f'[runner] update_drawer_state 실패 — DB 불일치, 수동 확인 필요: {e}'
                    )
                    try:
                        self._plc.set_error()
                    except Exception:
                        pass
        else:
            self._on_sequence_failure()
            if self._estop_triggered:
                # S-3: E-stop 상태에서는 홈 복귀 시퀀스 진입 금지 — actuator 명령 누출 차단
                # S-3: PLC 빨강 Solid + DB 로그 기록 필수
                self.get_logger().error(f'[runner] E-stop으로 시퀀스 중단: {self._seq_name} — 홈 복귀 생략')
                try:
                    self._plc.set_error()
                except Exception as e:
                    self.get_logger().error(f'[runner] E-stop PLC 오류 표시 실패: {e}')
                try:
                    if self._db is not None:
                        self._db.log_event(
                            tool_id=self._tool_id or 'unknown',
                            event_type='error',
                            track='A',
                            notes=f'estop_abort seq={self._seq_name}',
                        )
                except Exception as e:
                    self.get_logger().error(f'[runner] E-stop DB 로그 실패: {e}')
                self._is_moving_pub.publish(Bool(data=False))
                rclpy.shutdown()
                return
            else:
                self.get_logger().error(f'[runner] 시퀀스 실패: {self._seq_name} — 홈 복귀 시도')
            home_ok = self._engine.run_sequence(home_seq(), tool_id=self._tool_id)
            if not home_ok:
                self.get_logger().error('[runner] 홈 복귀 실패 — 수동 개입 필요')
                self._is_moving_pub.publish(Bool(data=False))  # S-7: 모든 종료 경로에서 발행
                rclpy.shutdown()
                return

        self._is_moving_pub.publish(Bool(data=False))
        rclpy.shutdown()

    def _on_sequence_failure(self) -> None:
        """E-5: 시퀀스 실패 시 DSR 정지 → PLC 오류 표시 + DB 시스템 이벤트 기록.

        엔진에 move_stop을 요청해 타임아웃된 move_line이 컨트롤러 큐에 남아 있는 경우
        MoveJoint 충돌을 방지한다(home_seq() 진입 전).
        """
        self._engine.stop_motion()
        try:
            self._plc.set_error()
        except Exception as e:
            self.get_logger().error(f'[runner] PLC 오류 표시 실패: {e}')
        if self._db is not None:
            try:
                self._db.log_system_event(
                    event_type='error',
                    severity='error',
                    track='A',
                    notes=(
                        f'sequence={self._seq_name} '
                        f'tool_id={self._tool_id or "N/A"} '
                        f'estop={self._estop_triggered}'
                    ),
                )
            except Exception as e:
                self.get_logger().error(f'[runner] DB 오류 로그 기록 실패: {e}')

    # ── 시퀀스 결정 ───────────────────────────────────────────────────────

    def _resolve_sequence(self, name: str) -> Optional[list]:
        if name == 'vision_fetch':
            return vision_fetch_seq(scan_j_deg=self._engine._fetch_scan_j_deg)

        if name == 'vision_return':
            return vision_return_seq(scan_j_deg=self._engine._return_scan_j_deg)

        if name in ('vision_open_0', 'vision_open_1'):
            layer = 0 if name == 'vision_open_0' else 1
            return vision_drawer_open_seq(layer)

        if name in ('vision_close_0', 'vision_close_1'):
            layer = 0 if name == 'vision_close_0' else 1
            return vision_drawer_close_seq(layer)

        mapping = {
            'home':          lambda: home_seq(),
            'open_0':        lambda: drawer_open_seq(0),
            'close_0':       lambda: drawer_close_seq(0),
            'open_1':        lambda: drawer_open_seq(1),
            'close_1':       lambda: drawer_close_seq(1),
            'open_0v2':      lambda: drawer_open_seq_v2(0),
            'close_0v2':     lambda: drawer_close_seq_v2(0),
            'open_1v2':      lambda: drawer_open_seq_v2(1),
            'close_1v2':     lambda: drawer_close_seq_v2(1),
            'fixed_fetch':   lambda: fixed_fetch_seq(),
            'socket_fetch':  lambda: socket_fetch_seq(),
            'socket_return': lambda: socket_return_seq(),
        }
        fn = mapping.get(name)
        return fn() if fn else None


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ToolboxSeqRunner()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except RuntimeError as e:
        logging.getLogger(__name__).error('[runner] 초기화 실패: %s', e)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
