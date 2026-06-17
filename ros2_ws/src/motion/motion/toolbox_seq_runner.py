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
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp
from dsr_msgs2.srv import GetCurrentPosx
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
    drawer_open_seq_v2,
    drawer_close_seq_v2,
    socket_fetch_seq,
    socket_return_seq,
    vision_fetch_seq,
    vision_return_seq,
    vision_drawer_open_seq,
    vision_drawer_close_seq,
)
from unit_actions.visual_servoing import (
    HandlePose,
    HandleServoController,
    ToolPose,
    ToolServoController,
    ServoConfig,
    ServoState,
    VelocityCommand as VSVelocityCommand,
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
        self._vel_l = self.get_parameter('vel_l').get_parameter_value().double_value
        self._acc_l = self.get_parameter('acc_l').get_parameter_value().double_value
        self._vel_r = self.get_parameter('vel_r').get_parameter_value().double_value
        self._acc_r = self.get_parameter('acc_r').get_parameter_value().double_value
        self._vel_j = self.get_parameter('vel_j').get_parameter_value().double_value
        self._acc_j = self.get_parameter('acc_j').get_parameter_value().double_value

        self._cb_group = ReentrantCallbackGroup()

        # S-3: E-stop 수신 플래그 — _on_estop 콜백과 _run_sequence 모두 접근
        self._estop_triggered: bool = False
        # S-3: VS에 주입할 Event — _on_estop에서 set()
        self._estop_event: threading.Event = threading.Event()

        # S-7: 기동 전 is_moving 상태 — Transient Local 구독으로 다른 노드의 retained 값 수신
        self._prev_is_moving: bool = False

        # VS (서랍 손잡이): /vision/handle_pose 구독으로 갱신
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입 (현재 geometry_msgs/PointStamped 가정)
        self._latest_handle: HandlePose = HandlePose()
        self._handle_lock: threading.Lock = threading.Lock()

        # VS (공구 접근): 탑뷰 XY + 그리퍼 캠 XYZ
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입·단위 (현재 PointStamped, m→mm 변환 적용)
        self._latest_top_tool: ToolPose = ToolPose()
        self._top_tool_lock: threading.Lock = threading.Lock()
        self._latest_gripper_tool: ToolPose = ToolPose()
        self._gripper_tool_lock: threading.Lock = threading.Lock()

        # VS (slot 반납): 탑뷰 slot 위치 XY
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입 확정
        self._latest_slot_top: ToolPose = ToolPose()
        self._slot_top_lock: threading.Lock = threading.Lock()

        p = f'/{ns}'
        self._movel_cli      = self.create_client(MoveLine,           f'{p}/motion/move_line',       callback_group=self._cb_group)
        self._movej_cli      = self.create_client(MoveJoint,          f'{p}/motion/move_joint',      callback_group=self._cb_group)
        self._stop_cli       = self.create_client(MoveStop,           f'{p}/motion/move_stop',       callback_group=self._cb_group)
        self._set_tcp_cli    = self.create_client(SetCurrentTcp,      f'{p}/tcp/set_current_tcp',    callback_group=self._cb_group)
        self._create_tcp_cli = self.create_client(ConfigCreateTcp,    f'{p}/tcp/config_create_tcp',  callback_group=self._cb_group)
        self._gripper_cli    = self.create_client(GripperSetPosition, '/gripper/set_position',       callback_group=self._cb_group)
        # VS: 현재 EE 포즈 조회 (DSR BASE 좌표계)
        self._get_posx_cli   = self.create_client(GetCurrentPosx,     f'{p}/aux_control/get_current_posx', callback_group=self._cb_group)

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

        # VS (서랍 손잡이): /vision/handle_pose 구독
        # ⚠️ 비전팀 확인 필요: 토픽명 확정
        self._handle_sub = self.create_subscription(
            PointStamped, '/vision/handle_pose', self._on_handle_pose, 10,
            callback_group=self._cb_group,
        )

        # VS (공구 접근): 탑뷰 D455f — rough XY (③번 이동용)
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입 확정
        self._top_tool_sub = self.create_subscription(
            PointStamped, '/vision/tool_top_pose', self._on_top_tool_pose, 10,
            callback_group=self._cb_group,
        )

        # VS (공구 접근): 그리퍼 캠 C270 — XY VS + Z 하강
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입 확정
        self._gripper_tool_sub = self.create_subscription(
            PointStamped, '/vision/tool_gripper_pose', self._on_gripper_tool_pose, 10,
            callback_group=self._cb_group,
        )

        # VS (slot 반납): 탑뷰 D455f — slot rough XY (⑧번 이동용)
        # ⚠️ 비전팀 확인 필요: 토픽명·메시지 타입 확정
        self._slot_top_sub = self.create_subscription(
            PointStamped, '/vision/slot_top_pose', self._on_slot_top_pose, 10,
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

        # E-4: config/toolbox.yaml vision_motion 섹션에서 좌표·범위 파라미터 로드
        self._load_toolbox_config()

        self._seq_name = seq_name
        self._done = False
        # 0.5s 후 _run_once 실행 — 그 사이 is_moving 구독 콜백이 retained 값을 수신할 수 있음
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb_group)

    # ── 설정 로딩 ─────────────────────────────────────────────────────────

    def _load_toolbox_config(self) -> None:
        """E-4: config/toolbox.yaml vision_motion 섹션에서 파라미터 로드."""
        import yaml
        cfg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../../config/toolbox.yaml',
        )
        try:
            with open(cfg_path, encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            vm = cfg.get('vision_motion', {})
            self._tool_approach_z_mm: float = float(vm.get('tool_approach_z_mm', 234.0))
            self._tool_approach_ori: list   = list(vm.get('tool_approach_ori', [53.23, 180.0, -38.07]))
            self._tool_descent_ori: list    = list(vm.get('tool_descent_ori', [48.74, -180.0, -42.55]))
            lim = vm.get('workspace_limits', {})
            x = lim.get('x', [50.0, 800.0])
            y = lim.get('y', [-600.0, 600.0])
            z = lim.get('z', [-5.0, 700.0])
            self._vis_x_min, self._vis_x_max = float(x[0]), float(x[1])
            self._vis_y_min, self._vis_y_max = float(y[0]), float(y[1])
            self._vis_z_min, self._vis_z_max = float(z[0]), float(z[1])
            # 공구별 grasp_z_mm 딕셔너리 로드
            self._grasp_z_map: dict[str, float] = {
                t['tool_id']: float(t['grasp_z_mm'])
                for t in cfg.get('tools', [])
                if 'grasp_z_mm' in t
            }
            self.get_logger().info(
                f'[runner] toolbox.yaml 로드 완료 — '
                f'approach_z={self._tool_approach_z_mm}mm '
                f'ws_x=[{self._vis_x_min},{self._vis_x_max}] '
                f'grasp_z_map={self._grasp_z_map}'
            )
        except Exception as e:
            self.get_logger().error(f'[runner] toolbox.yaml 로드 실패: {e} — 기본값 사용')
            self._tool_approach_z_mm = 234.0
            self._tool_approach_ori  = [53.23, 180.0, -38.07]
            self._tool_descent_ori   = [48.74, -180.0, -42.55]
            self._vis_x_min, self._vis_x_max = 50.0, 800.0
            self._vis_y_min, self._vis_y_max = -600.0, 600.0
            self._vis_z_min, self._vis_z_max = -5.0, 700.0
            self._grasp_z_map = {}

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _on_is_moving(self, msg: Bool) -> None:
        """S-7: is_moving 토픽 수신 — 기동 전 다른 시퀀스 실행 여부 확인용."""
        self._prev_is_moving = msg.data

    def _on_estop(self, msg: Bool) -> None:
        """S-3: E-stop 수신 — 플래그 세팅 + Event set + DSR move_stop 즉시 요청."""
        if msg.data and not self._estop_triggered:
            self._estop_triggered = True
            self._estop_event.set()   # VS 루프에 즉시 전달
            self.get_logger().error('[runner] E-stop 수신 — 모션 즉시 중단 요청')
            if self._stop_cli.service_is_ready():
                fut = self._stop_cli.call_async(MoveStop.Request())
                fut.add_done_callback(self._on_move_stop_result)
            else:
                self.get_logger().error('[runner] move_stop 서비스 미준비 — DSR 정지 명령 전송 불가 (수동 확인 필요)')

    def _on_move_stop_result(self, future) -> None:
        """S-3: move_stop 응답 확인 — 실패 시 경고 로그."""
        try:
            res = future.result()
            if res is None or not res.success:
                self.get_logger().error('[runner] move_stop 응답 실패 — DSR 정지 미확인 (수동 확인 필요)')
            else:
                self.get_logger().info('[runner] move_stop 응답 확인')
        except Exception as e:
            self.get_logger().error(f'[runner] move_stop 응답 처리 오류: {e}')

    def _on_handle_pose(self, msg: PointStamped) -> None:
        """VS: 손잡이 중심 좌표 수신 (robot base frame, m → mm 변환)."""
        with self._handle_lock:
            self._latest_handle = HandlePose(
                x=msg.point.x * 1000.0,   # m → mm
                z=msg.point.z * 1000.0,
                valid=True,
            )

    def _on_top_tool_pose(self, msg: PointStamped) -> None:
        """탑뷰 D455f 공구 좌표 수신 — ③ MOVE_L_TOP_XY 에서 rough XY 로 사용."""
        with self._top_tool_lock:
            self._latest_top_tool = ToolPose(
                x=msg.point.x * 1000.0,
                y=msg.point.y * 1000.0,
                z=msg.point.z * 1000.0,
                valid=True,
            )

    def _on_gripper_tool_pose(self, msg: PointStamped) -> None:
        """그리퍼 캠 C270 공구 좌표 수신 — ④⑨ VS XY 정렬 + ⑤⑩ Z 하강에 사용."""
        with self._gripper_tool_lock:
            self._latest_gripper_tool = ToolPose(
                x=msg.point.x * 1000.0,
                y=msg.point.y * 1000.0,
                z=msg.point.z * 1000.0,
                valid=True,
            )

    def _on_slot_top_pose(self, msg: PointStamped) -> None:
        """탑뷰 D455f slot 위치 수신 — ⑧ MOVE_L_SLOT_XY 에서 rough XY 로 사용."""
        with self._slot_top_lock:
            self._latest_slot_top = ToolPose(
                x=msg.point.x * 1000.0,
                y=msg.point.y * 1000.0,
                z=msg.point.z * 1000.0,
                valid=True,
            )

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
                self.get_logger().error(f'[runner] E-stop으로 시퀀스 중단: {self._seq_name} — 홈 복귀 생략')
                self._is_moving_pub.publish(Bool(data=False))
                rclpy.shutdown()
                return
            else:
                self.get_logger().error(f'[runner] 시퀀스 실패: {self._seq_name} — 홈 복귀 시도')
            home_ok = self._run_sequence(home_seq())
            if not home_ok:
                self.get_logger().error('[runner] 홈 복귀 실패 — 수동 개입 필요')
                self._is_moving_pub.publish(Bool(data=False))  # S-7: 모든 종료 경로에서 발행
                rclpy.shutdown()
                return

        self._is_moving_pub.publish(Bool(data=False))
        rclpy.shutdown()

    def _on_sequence_failure(self) -> None:
        """E-5: 시퀀스 실패 시 DSR 정지 → PLC 오류 표시 + DB 시스템 이벤트 기록.

        home_seq() 진입 전 move_stop을 먼저 호출해 타임아웃된 move_line이
        컨트롤러 큐에 남아 있는 경우 MoveJoint 충돌을 방지한다.
        """
        if self._stop_cli.service_is_ready():
            try:
                fut = self._stop_cli.call_async(MoveStop.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                self.get_logger().info('[runner] 시퀀스 실패 — DSR move_stop 전송')
                time.sleep(0.3)  # DSR 감속 완료 대기 — HIL 실측 후 조정
            except Exception as e:
                self.get_logger().error(f'[runner] move_stop 실패 (무시): {e}')
        else:
            self.get_logger().warn('[runner] move_stop 서비스 미준비 — DSR 정지 생략')
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
            (x, self._vis_x_min, self._vis_x_max, 'x'),
            (y, self._vis_y_min, self._vis_y_max, 'y'),
            (z, self._vis_z_min, self._vis_z_max, 'z'),
        ]:
            if not (lo <= val <= hi):
                self.get_logger().error(
                    f'[runner] {label} {axis}={val:.1f}mm 범위 초과 [{lo}, {hi}] — 실행 거부'
                )
                return False
        return True

    def _resolve_sequence(self, name: str) -> Optional[list]:
        if name == 'vision_fetch':
            return vision_fetch_seq()  # 좌표는 토픽에서 실시간 수신 — 파라미터 불필요

        if name == 'vision_return':
            return vision_return_seq()  # 좌표는 토픽에서 실시간 수신 — 파라미터 불필요

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
        elif step.kind == StepKind.VISUAL_SERVO_XZ:
            return self._exec_visual_servo()
        elif step.kind == StepKind.MOVE_L_TOP_XY:
            return self._exec_move_l_top_xy()
        elif step.kind == StepKind.VISUAL_SERVO_XY:
            return self._exec_visual_servo_xy()
        elif step.kind == StepKind.MOVE_L_TOOL_XYZ:
            return self._exec_move_l_tool_xyz()
        elif step.kind == StepKind.MOVE_L_SLOT_XY:
            return self._exec_move_l_slot_xy()
        elif step.kind == StepKind.WAIT_VISION_TOP_XY:
            return self._exec_wait_vision_top_xy()
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
            res = fut.result()
            if res is None or not getattr(res, 'success', True):
                msg = getattr(res, 'message', 'timeout')
                if self._mode == 'virtual':
                    # virtual 모드: home_on_start가 이미 TCP를 설정했으므로 non-fatal
                    self.get_logger().warn(
                        f'[runner] set_current_tcp 응답 실패 (virtual — 무시): name={name} msg={msg}'
                    )
                else:
                    self.get_logger().error(
                        f'[runner] set_current_tcp 응답 실패: name={name} msg={msg}'
                    )
                    return False
            else:
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

    # ── Visual Servoing ───────────────────────────────────────────────────────

    def _exec_visual_servo(self) -> bool:
        """VISUAL_SERVO_XZ 스텝 실행 — HandleServoController 루프."""
        cfg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../../config/visual_servo.yaml',
        )
        try:
            cfg = ServoConfig.load_from_yaml(cfg_path, section="handle")
        except Exception as e:
            self.get_logger().error(f'  visual_servo.yaml 로드 실패: {e}')
            return False

        ctrl = HandleServoController(
            cfg=cfg,
            get_handle=self._get_latest_handle,
            get_ee_pose=self._get_ee_pose_mm,
            estop_event=self._estop_event,
        )

        _DT = 0.033  # 30 Hz
        self.get_logger().info('  [VS] 시작')

        while not ctrl.is_terminal():
            if self._estop_triggered:
                return False

            try:
                cmd = ctrl.tick()
            except RuntimeError as e:
                self.get_logger().error(f'  [VS] EE 위치 조회 실패 — 루프 중단: {e}')
                return False

            if cmd.stop:
                time.sleep(_DT)
                continue

            if cmd.vx != 0.0 or cmd.vz != 0.0:
                ok = self._movel_delta(cmd.vx, cmd.vz, _DT)
                if not ok:
                    self.get_logger().error('  [VS] movel_delta 실패')
                    return False

            time.sleep(_DT)

        if ctrl.state == ServoState.DONE:
            self.get_logger().info('  [VS] XZ 정렬 완료')
            return True

        self.get_logger().error(f'  [VS] 실패: {ctrl.state.name}')
        return False

    # ── 공구 접근 VS (vision_fetch) ───────────────────────────────────────────

    def _exec_wait_vision_top_xy(self, timeout_sec: float = 5.0) -> bool:
        """④: 탑뷰 캐시 초기화 후 /vision/tool_top_pose 신규 수신 대기."""
        with self._top_tool_lock:
            self._latest_top_tool = ToolPose(x=0.0, y=0.0, z=0.0, valid=False)
        self.get_logger().info('  [WAIT_VIS] 탑뷰 공구 좌표 대기 중...')
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self._top_tool_lock:
                if self._latest_top_tool.valid:
                    p = self._latest_top_tool
                    self.get_logger().info(
                        f'  [WAIT_VIS] 수신 완료 → ({p.x:.1f}, {p.y:.1f}) mm'
                    )
                    return True
            time.sleep(0.05)
        self.get_logger().error(
            f'  [WAIT_VIS] 탑뷰 좌표 수신 타임아웃 ({timeout_sec:.0f}s) — /vision/tool_top_pose 확인 필요'
        )
        return False

    def _exec_move_l_top_xy(self) -> bool:
        """⑤⑧: 탑뷰 D455f XY + self._tool_approach_z_mm 로 이동."""
        with self._top_tool_lock:
            pose = ToolPose(
                x=self._latest_top_tool.x,
                y=self._latest_top_tool.y,
                z=self._latest_top_tool.z,
                valid=self._latest_top_tool.valid,
            )
        if not pose.valid:
            self.get_logger().error('  [TOP_XY] 탑뷰 공구 좌표 미수신 (/vision/tool_top_pose)')
            return False
        if not self._check_vision_coords('TOP_XY', pose.x, pose.y, self._tool_approach_z_mm):
            return False
        pos = [pose.x, pose.y, self._tool_approach_z_mm] + list(self._tool_approach_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self.get_logger().info(f'  [TOP_XY] → ({pose.x:.1f}, {pose.y:.1f}, {self._tool_approach_z_mm:.1f}) mm')
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_visual_servo_xy(self) -> bool:
        """④: 그리퍼 캠 C270 XY P제어 수렴 — ToolServoController."""
        cfg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../../config/visual_servo.yaml',
        )
        try:
            cfg = ServoConfig.load_from_yaml(cfg_path, section="tool")
        except Exception as e:
            self.get_logger().error(f'  [VS_XY] visual_servo.yaml 로드 실패: {e}')
            return False

        ctrl = ToolServoController(
            cfg=cfg,
            get_tool=self._get_latest_gripper_tool,
            get_ee_pose=self._get_ee_pose_mm,
            estop_event=self._estop_event,
        )

        _DT = 0.033  # 30 Hz
        self.get_logger().info('  [VS_XY] 시작')

        while not ctrl.is_terminal():
            if self._estop_triggered:
                return False

            try:
                cmd = ctrl.tick()
            except RuntimeError as e:
                self.get_logger().error(f'  [VS_XY] EE 위치 조회 실패 — 루프 중단: {e}')
                return False

            if cmd.stop:
                time.sleep(_DT)
                continue

            if cmd.vx != 0.0 or cmd.vy != 0.0:
                ok = self._movel_delta_xy(cmd.vx, cmd.vy, _DT)
                if not ok:
                    self.get_logger().error('  [VS_XY] movel_delta_xy 실패')
                    return False

            time.sleep(_DT)

        if ctrl.state == ServoState.DONE:
            self.get_logger().info('  [VS_XY] XY 정렬 완료')
            return True

        self.get_logger().error(f'  [VS_XY] 실패: {ctrl.state.name}')
        return False

    def _exec_move_l_tool_xyz(self) -> bool:
        """⑥: 그리퍼 캠 XY + toolbox.yaml grasp_z_mm(tool_id별) 로 공구 위치까지 하강."""
        with self._gripper_tool_lock:
            pose = ToolPose(
                x=self._latest_gripper_tool.x,
                y=self._latest_gripper_tool.y,
                z=self._latest_gripper_tool.z,
                valid=self._latest_gripper_tool.valid,
            )
        if not pose.valid:
            self.get_logger().error('  [TOOL_XYZ] 그리퍼 캠 공구 좌표 미수신 (/vision/tool_gripper_pose)')
            return False

        # Z: toolbox.yaml grasp_z_mm 우선 사용, 없으면 그리퍼 캠 Z 폴백
        grasp_z = self._grasp_z_map.get(self._tool_id)
        if grasp_z is not None:
            z = grasp_z
            self.get_logger().info(f'  [TOOL_XYZ] Z = {z:.2f}mm (toolbox.yaml, tool_id={self._tool_id})')
        else:
            z = pose.z
            self.get_logger().warn(
                f'  [TOOL_XYZ] tool_id={self._tool_id!r} grasp_z_mm 미등록 — 그리퍼 캠 Z 사용: {z:.2f}mm'
            )

        if not self._check_vision_coords('TOOL_XYZ', pose.x, pose.y, z):
            return False
        pos = [pose.x, pose.y, z] + list(self._tool_descent_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self.get_logger().info(f'  [TOOL_XYZ] → ({pose.x:.1f}, {pose.y:.1f}, {z:.1f}) mm')
        return self._movel(step, DR_MV_MOD_ABS)

    def _get_latest_gripper_tool(self) -> ToolPose:
        with self._gripper_tool_lock:
            return ToolPose(
                x=self._latest_gripper_tool.x,
                y=self._latest_gripper_tool.y,
                z=self._latest_gripper_tool.z,
                valid=self._latest_gripper_tool.valid,
            )

    def _exec_move_l_slot_xy(self) -> bool:
        """⑧⑫: 탑뷰 D455f slot XY + self._tool_approach_z_mm 로 이동."""
        with self._slot_top_lock:
            pose = ToolPose(
                x=self._latest_slot_top.x,
                y=self._latest_slot_top.y,
                z=self._latest_slot_top.z,
                valid=self._latest_slot_top.valid,
            )
        if not pose.valid:
            self.get_logger().error('  [SLOT_XY] slot 좌표 미수신 (/vision/slot_top_pose)')
            return False
        # Z 이동 목표는 토픽값이 아닌 고정 approach 높이(tool_approach_z_mm) — pose.z 미사용
        if not self._check_vision_coords('SLOT_XY', pose.x, pose.y, self._tool_approach_z_mm):
            return False
        pos = [pose.x, pose.y, self._tool_approach_z_mm] + list(self._tool_approach_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self.get_logger().info(f'  [SLOT_XY] → ({pose.x:.1f}, {pose.y:.1f}, {self._tool_approach_z_mm:.1f}) mm')
        return self._movel(step, DR_MV_MOD_ABS)

    def _movel_delta_xy(self, vx: float, vy: float, dt: float) -> bool:
        """속도(mm/s) × dt(s) = XY delta(mm) RELATIVE MoveL."""
        req = MoveLine.Request()
        req.pos        = [vx * dt, vy * dt, 0.0, 0.0, 0.0, 0.0]
        req.vel        = [self._vel_l, self._vel_r]
        req.acc        = [self._acc_l, self._acc_r]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movel_cli.call_async(req)
        # S-4: PLC Watchdog(0.5s timeout)은 plc_node 독립 타이머로 운용.
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.45)
        res = fut.result()
        return res is not None and res.success

    def _get_latest_handle(self) -> HandlePose:
        """스레드 안전하게 최신 손잡이 좌표 반환."""
        with self._handle_lock:
            return HandlePose(
                x=self._latest_handle.x,
                z=self._latest_handle.z,
                valid=self._latest_handle.valid,
            )

    def _get_ee_pose_mm(self) -> tuple[float, float, float]:
        """현재 EE 포즈 (DSR BASE 좌표계, mm) 반환. 실패 시 RuntimeError (E-5: silent fallback 금지)."""
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        fut = self._get_posx_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError('get_current_posx 서비스 실패 — VS 루프 즉시 중단')
        pos = res.task_pos_info[0].data  # [x, y, z, rx, ry, rz]
        return (pos[0], pos[1], pos[2])

    def _movel_delta(self, vx: float, vz: float, dt: float) -> bool:
        """속도(mm/s) × dt(s) = 위치 delta(mm) 로 변환해 RELATIVE MoveL 실행."""
        req = MoveLine.Request()
        req.pos        = [vx * dt, 0.0, vz * dt, 0.0, 0.0, 0.0]
        req.vel        = [self._vel_l, self._vel_r]
        req.acc        = [self._acc_l, self._acc_r]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movel_cli.call_async(req)
        # S-4: PLC Watchdog(0.5s timeout)은 plc_node 독립 타이머로 운용 — 이 블로킹과 무관.
        # timeout을 0.45s로 제한해 Watchdog 경계 내에서 서비스 무응답을 감지.
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.45)
        res = fut.result()
        return res is not None and res.success


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
