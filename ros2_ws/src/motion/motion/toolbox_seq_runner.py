"""toolbox_seq_runner.py
────────────────────────
toolbox_motion.py 시퀀스를 virtual/real 모드에서 실행하는 테스트 노드.

실행:
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=open_0
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_fetch \\
    -p tool_id:=screwdriver_phillips_small -p vision_x_mm:=300.0 ...

  sequence 옵션:
    open_0  / close_0  — layer 0 (1층 서랍)
    open_1  / close_1  — layer 1 (2층 서랍)
    socket_fetch / socket_return — 소켓 공구 (tool_id 필수)
    vision_fetch / vision_return — 비전 기반 공구 (tool_id 필수)

단위: toolbox_motion.py 좌표는 DSR 네이티브(mm/deg) → move_line/move_joint 직접 전달.
"""

import logging
import os
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp
from interfaces.srv import GripperSetPosition


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
    StepKind,
    Step,
    VEL_L, ACC_L, VEL_R, ACC_R, VEL_J, ACC_J,
    home_seq,
    drawer_open_seq,
    drawer_close_seq,
    socket_fetch_seq,
    socket_return_seq,
    vision_fetch_seq,
    vision_return_seq,
    vision_drawer_open_seq,
    vision_drawer_close_seq,
)
from db_core.client import DBClient, DBError, DBCacheExpiredError
from plc_core.client import PLCClient

DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# S-7 / S-3: Transient Local QoS — 구독 시 최신 retained 값 즉시 수신
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# 비전 좌표 허용 범위 (DSR BASE 좌표계, mm)
# 실제 비전 좌표 수신 후 config/toolbox.yaml 이관 예정 (E-4)
_VIS_X_MIN, _VIS_X_MAX = 50.0,  800.0
_VIS_Y_MIN, _VIS_Y_MAX = -600.0, 600.0
_VIS_Z_MIN, _VIS_Z_MAX = -5.0,   700.0

# S-2: DB gate 적용 대상 시퀀스
_FETCH_SEQS  = {'vision_fetch', 'socket_fetch'}
_RETURN_SEQS = {'vision_return', 'socket_return'}
_GATE_SEQS   = _FETCH_SEQS | _RETURN_SEQS


class ToolboxSeqRunner(Node):

    def __init__(self) -> None:
        super().__init__('toolbox_seq_runner')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('sequence', 'open_0')
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
        self._vel_l = self.get_parameter('vel_l').get_parameter_value().double_value
        self._acc_l = self.get_parameter('acc_l').get_parameter_value().double_value
        self._vel_r = self.get_parameter('vel_r').get_parameter_value().double_value
        self._acc_r = self.get_parameter('acc_r').get_parameter_value().double_value
        self._vel_j = self.get_parameter('vel_j').get_parameter_value().double_value
        self._acc_j = self.get_parameter('acc_j').get_parameter_value().double_value

        self._cb_group = ReentrantCallbackGroup()

        # S-3: E-stop 수신 플래그 — _on_estop 콜백과 _run_sequence 모두 접근
        self._estop_triggered: bool = False

        # S-7: 기동 전 is_moving 상태 — Transient Local 구독으로 다른 노드의 retained 값 수신
        self._prev_is_moving: bool = False

        p = f'/{ns}'
        self._movel_cli      = self.create_client(MoveLine,           f'{p}/motion/move_line',       callback_group=self._cb_group)
        self._movej_cli      = self.create_client(MoveJoint,          f'{p}/motion/move_joint',      callback_group=self._cb_group)
        self._stop_cli       = self.create_client(MoveStop,           f'{p}/motion/move_stop',       callback_group=self._cb_group)
        self._set_tcp_cli    = self.create_client(SetCurrentTcp,      f'{p}/tcp/set_current_tcp',    callback_group=self._cb_group)
        self._create_tcp_cli = self.create_client(ConfigCreateTcp,    f'{p}/tcp/config_create_tcp',  callback_group=self._cb_group)
        self._gripper_cli    = self.create_client(GripperSetPosition, '/gripper/set_position',       callback_group=self._cb_group)

        self.get_logger().info('[runner] 서비스 대기 중...')
        for cli, name in [
            (self._movel_cli,   'move_line'),
            (self._movej_cli,   'move_joint'),
            (self._set_tcp_cli, 'set_current_tcp'),
            (self._gripper_cli, 'gripper/set_position'),
        ]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'[runner] {name} 없음 — bringup 먼저 실행')
                raise RuntimeError(f'{name} 서비스 없음')
            self.get_logger().info(f'[runner] {name} 연결됨')

        # S-7: is_moving pub/sub — Transient Local로 retained 값 유지·수신
        # 구독을 먼저 생성해 다른 노드의 retained True를 _run_once 전에 수신한다.
        # True publish는 is_moving 확인 후 _run_once에서 수행한다.
        self._is_moving_pub = self.create_publisher(Bool, '/motion/is_moving', _LATCHED_QOS)
        self._is_moving_sub = self.create_subscription(
            Bool, '/motion/is_moving', self._on_is_moving, _LATCHED_QOS,
            callback_group=self._cb_group,
        )

        # S-3: E-stop 구독 — /plc/e_stop (interfaces.md §4, Reliable + Transient Local)
        self._estop_sub = self.create_subscription(
            Bool, '/plc/e_stop', self._on_estop, _LATCHED_QOS,
            callback_group=self._cb_group,
        )

        # S-2: DB 클라이언트 — fetch/return 실행 전 feasibility 판정
        self._db: DBClient | None = None
        try:
            self._db = DBClient(db_path)
            self._db.connect()
        except Exception as e:
            self.get_logger().warn(f'[runner] DB 연결 실패 (fetch/return 실행 불가): {e}')

        # E-5: PLC 클라이언트 — 시퀀스 실패 시 오류 상태 표시
        self._plc = PLCClient()
        self._plc.connect()

        self._seq_name = seq_name
        self._done = False
        # 0.5s 후 _run_once 실행 — 그 사이 is_moving 구독 콜백이 retained 값을 수신할 수 있음
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb_group)

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _on_is_moving(self, msg: Bool) -> None:
        """S-7: is_moving 토픽 수신 — 기동 전 다른 시퀀스 실행 여부 확인용."""
        self._prev_is_moving = msg.data

    def _on_estop(self, msg: Bool) -> None:
        """S-3: E-stop 수신 — 플래그 세팅 + DSR move_stop 즉시 요청."""
        if msg.data and not self._estop_triggered:
            self._estop_triggered = True
            self.get_logger().error('[runner] E-stop 수신 — 모션 즉시 중단 요청')
            if self._stop_cli.service_is_ready():
                self._stop_cli.call_async(MoveStop.Request())

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
                'socket_fetch socket_return vision_fetch vision_return '
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

        # S-7: 시퀀스 시작 직전 is_moving=True 발행
        self._is_moving_pub.publish(Bool(data=True))

        if not self._set_tcp(self._tcp_name):
            self.get_logger().error('[runner] TCP 설정 실패 — 시퀀스 중단')
            self._on_sequence_failure()
            self._is_moving_pub.publish(Bool(data=False))
            rclpy.shutdown()
            return

        self.get_logger().info(f'[runner] 시퀀스 시작: {self._seq_name} ({len(seq)} steps)')
        ok = self._run_sequence(seq)

        if ok:
            self.get_logger().info(f'[runner] 시퀀스 완료: {self._seq_name}')
        else:
            if self._estop_triggered:
                self.get_logger().error(f'[runner] E-stop으로 시퀀스 중단: {self._seq_name}')
            else:
                self.get_logger().error(f'[runner] 시퀀스 실패: {self._seq_name} — 홈 복귀 시도')
            self._on_sequence_failure()
            home_ok = self._run_sequence(home_seq())
            if not home_ok:
                self.get_logger().error('[runner] 홈 복귀 실패 — 수동 개입 필요')
                rclpy.shutdown()
                return

        self._is_moving_pub.publish(Bool(data=False))
        rclpy.shutdown()

    def _on_sequence_failure(self) -> None:
        """E-5: 시퀀스 실패 시 PLC 오류 표시 + DB 시스템 이벤트 기록."""
        try:
            self._plc.set_error()
        except Exception as e:
            self.get_logger().error(f'[runner] PLC 오류 표시 실패: {e}')
        if self._db is not None:
            try:
                self._db.log_system_event(
                    event_type='error',
                    severity='error',
                    track='B',
                    notes=(
                        f'sequence={self._seq_name} '
                        f'tool_id={self._tool_id or "N/A"} '
                        f'estop={self._estop_triggered}'
                    ),
                )
            except Exception as e:
                self.get_logger().error(f'[runner] DB 오류 로그 기록 실패: {e}')

    # ── 시퀀스 결정 ───────────────────────────────────────────────────────

    def _check_vision_coords(self, label: str, x: float, y: float, z: float) -> bool:
        if x == 0.0 and y == 0.0 and z == 0.0:
            self.get_logger().error(f'[runner] {label} 좌표 미설정 (0,0,0) — 실행 거부')
            return False
        for val, lo, hi, axis in [
            (x, _VIS_X_MIN, _VIS_X_MAX, 'x'),
            (y, _VIS_Y_MIN, _VIS_Y_MAX, 'y'),
            (z, _VIS_Z_MIN, _VIS_Z_MAX, 'z'),
        ]:
            if not (lo <= val <= hi):
                self.get_logger().error(
                    f'[runner] {label} {axis}={val:.1f}mm 범위 초과 [{lo}, {hi}] — 실행 거부'
                )
                return False
        return True

    def _resolve_sequence(self, name: str) -> Optional[list]:
        if name == 'vision_fetch':
            if not self._check_vision_coords('vision', self._vision_x, self._vision_y, self._vision_z):
                return None
            return vision_fetch_seq(self._vision_x, self._vision_y, self._vision_z)

        if name == 'vision_return':
            if not self._check_vision_coords('bottom', self._bottom_x, self._bottom_y, self._bottom_z):
                return None
            if not self._check_vision_coords('slot', self._slot_x, self._slot_y, self._slot_z):
                return None
            return vision_return_seq(
                self._bottom_x, self._bottom_y, self._bottom_z,
                self._slot_x,   self._slot_y,   self._slot_z,
            )

        if name in ('vision_open_0', 'vision_open_1'):
            if not self._check_vision_coords('approach', self._approach_x, self._approach_y, self._approach_z):
                return None
            layer = 0 if name == 'vision_open_0' else 1
            return vision_drawer_open_seq(layer, self._approach_x, self._approach_y, self._approach_z)

        if name in ('vision_close_0', 'vision_close_1'):
            if not self._check_vision_coords('approach', self._approach_x, self._approach_y, self._approach_z):
                return None
            layer = 0 if name == 'vision_close_0' else 1
            return vision_drawer_close_seq(layer, self._approach_x, self._approach_y, self._approach_z)

        mapping = {
            'home':          lambda: home_seq(),
            'open_0':        lambda: drawer_open_seq(0),
            'close_0':       lambda: drawer_close_seq(0),
            'open_1':        lambda: drawer_open_seq(1),
            'close_1':       lambda: drawer_close_seq(1),
            'socket_fetch':  lambda: socket_fetch_seq(),
            'socket_return': lambda: socket_return_seq(),
        }
        fn = mapping.get(name)
        return fn() if fn else None

    # ── 시퀀스 실행 ───────────────────────────────────────────────────────

    def _run_sequence(self, steps: list) -> bool:
        for i, step in enumerate(steps):
            # S-3: E-stop 수신 시 다음 스텝 진입 전 중단
            if self._estop_triggered:
                self.get_logger().error(f'  E-stop — step {i+1}/{len(steps)} 진입 전 중단')
                return False
            self.get_logger().info(f'  step {i+1}/{len(steps)}: {step.kind.name}')
            ok = self._exec_step(step)
            if not ok:
                self.get_logger().error(f'  step {i+1} 실패 — 중단')
                return False
        return True

    def _exec_step(self, step: Step) -> bool:
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
        self.get_logger().warn(f'  알 수 없는 StepKind: {step.kind}')
        return False

    # ── DSR 서비스 호출 ───────────────────────────────────────────────────

    def _set_tcp(self, name: str) -> bool:
        try:
            if self._create_tcp_cli.service_is_ready():
                create_req = ConfigCreateTcp.Request()
                create_req.name = name
                create_req.pos  = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
                fut = self._create_tcp_cli.call_async(create_req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                self.get_logger().info(f'[runner] TCP 등록 완료: {name} pos=[0,0,160,0,0,0]')
            else:
                self.get_logger().warn('[runner] config_create_tcp 서비스 미준비 — 건너뜀')

            # set_current_tcp는 시퀀스 실행의 전제 조건 — 미준비 시 중단
            if not self._set_tcp_cli.service_is_ready():
                self.get_logger().error('[runner] set_current_tcp 서비스 미준비 — 시퀀스 중단')
                return False

            set_req = SetCurrentTcp.Request()
            set_req.name = name
            fut = self._set_tcp_cli.call_async(set_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            self.get_logger().info(f'[runner] TCP 활성화 완료: {name}')
            return True

        except Exception as e:
            self.get_logger().error(f'[runner] TCP 설정 예외: {e}')
            return False

    def _movel(self, step: Step, mode: int) -> bool:
        req = MoveLine.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = [min(step.vel, self._vel_l) if step.vel is not None else self._vel_l,
                          min(step.vel, self._vel_r) if step.vel is not None else self._vel_r]
        req.acc        = [min(step.acc, self._acc_l) if step.acc is not None else self._acc_l,
                          min(step.acc, self._acc_r) if step.acc is not None else self._acc_r]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = mode
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movel_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            self.get_logger().error(f'  move_line timeout (30s) — 서비스 무응답: pos={step.pose}')
            return False
        time.sleep(0.2)
        if not res.success:
            self.get_logger().error(f'  move_line 실패: pos={step.pose}')
        return bool(res.success)

    def _movej(self, step: Step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = min(step.vel, self._vel_j) if step.vel is not None else self._vel_j
        req.acc        = min(step.acc, self._acc_j) if step.acc is not None else self._acc_j
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movej_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            self.get_logger().error(f'  move_joint timeout (30s) — 서비스 무응답: pos={step.pose}')
            return False
        time.sleep(0.2)
        if not res.success:
            self.get_logger().error(f'  move_joint 실패: pos={step.pose}')
        return bool(res.success)

    def _grip(self, step: Step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0
        current = 400 if pulse > 450 else 0  # grip: 400mA, open/release: gripper_node 기본값

        req = GripperSetPosition.Request()
        req.position    = pulse
        req.current     = current
        req.timeout_sec = 0.0

        fut = self._gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            msg = res.message if res else 'timeout'
            self.get_logger().error(f'  gripper set_position 실패: {msg}')
            return False
        self.get_logger().info(f'  gripper ok — pos={res.final_position} cur={res.final_current}')
        time.sleep(0.1)
        return True


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
