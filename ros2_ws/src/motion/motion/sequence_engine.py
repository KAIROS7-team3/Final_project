"""sequence_engine.py
────────────────────────
toolbox_motion.py 시퀀스를 실행하는 노드 비의존 재사용 엔진.

`SequenceEngine`은 DSR 서비스 호출 + 비전 토픽 구독 + E-stop 감시 + step 실행을
모두 소유한다. DB/PLC/is_moving/시퀀스 선택 같은 정책 결정은 호출자(러너·액션 서버)
책임이며 엔진은 알지 못한다.

- ExecutePhase 액션 서버와 CLI HIL 러너(`toolbox_seq_runner.py`)가 공유한다.
- rclpy 사용 가능(motion은 ROS2 패키지). 단 DBClient/PLCClient는 포함하지 않는다.

단위: toolbox_motion.py 좌표는 DSR 네이티브(mm/deg) → move_line/move_joint 직접 전달.
"""

import math
import os
import sys
import threading
import time
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp, SetRobotMode
from dsr_msgs2.srv import GetCurrentPosx
from interfaces.srv import GripperSetPosition


def _add_unit_actions_to_path() -> None:
    """unit_actions/ 는 ros2_ws 밖(레포 루트)에 있고 ROS2 패키지가 아니라 colcon이
    설치하지 않으므로 레포 루트를 sys.path에 추가한다. 엔진을 직접 import 하는
    호출자(액션 서버·pytest)에서도 동작하도록 엔진이 자체적으로 보장한다
    (toolbox_seq_runner와 동일 패턴)."""
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
        "환경변수로 지정하세요."
    )


_add_unit_actions_to_path()

from unit_actions.toolbox_motion import (  # noqa: E402
    StepKind,
    Step,
    VEL_L, ACC_L, VEL_R, ACC_R, VEL_J, ACC_J,
    PULSE_GRIP_TOOL,
)
from unit_actions.visual_servoing import (  # noqa: E402
    HandlePose,
    HandleServoController,
    ToolPose,
    ToolServoController,
    ServoConfig,
    ServoState,
)

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


class SequenceEngine:
    """toolbox_motion 시퀀스 실행 엔진 (노드 비의존, DB/PLC 미포함)."""

    def __init__(
        self,
        node: Node,
        *,
        robot_ns: str,
        tcp_name: str,
        config_path: str,
        mode: str = "virtual",
        vel_l: float = VEL_L,
        acc_l: float = ACC_L,
        vel_r: float = VEL_R,
        acc_r: float = ACC_R,
        vel_j: float = VEL_J,
        acc_j: float = ACC_J,
        estop_event: Optional[threading.Event] = None,
    ) -> None:
        self._node = node
        self._robot_ns = robot_ns
        self._tcp_name = tcp_name
        self._config_path = config_path
        self._mode = mode
        # S-5/E-4: 속도·가속도 상한 — 호출자가 config 값으로 오버라이드 가능
        self._vel_l = vel_l
        self._acc_l = acc_l
        self._vel_r = vel_r
        self._acc_r = acc_r
        self._vel_j = vel_j
        self._acc_j = acc_j

        # tool_id는 phase마다 다르므로 run_sequence 인자로 받아 현재 값으로 저장한 뒤 _exec_*가 사용
        self._tool_id: str = ""

        self._cb_group = ReentrantCallbackGroup()

        # S-3: E-stop 수신 플래그 — _on_estop 콜백과 run_sequence 모두 접근
        self._estop_triggered: bool = False
        # S-3: VS에 주입할 Event — _on_estop에서 set(). 호출자가 공유 Event 주입 가능.
        self._estop_event: threading.Event = estop_event if estop_event is not None else threading.Event()

        # VS (서랍 손잡이): /vision/handle_pose 구독으로 갱신
        self._latest_handle: HandlePose = HandlePose()
        self._handle_lock: threading.Lock = threading.Lock()

        # fetch 스캔 자세에서 찍은 공구 좌표 (XY + rz) — PoseStamped
        self._latest_tool_gripper: ToolPose = ToolPose()
        # WAIT_VISION 시점에 고정 캡처 — MOVE_L_TOP_XY / STAGING_XYZ가 동일 pose 사용
        self._staged_gripper_pose: ToolPose = ToolPose()
        self._latest_gripper_is_top: bool = False  # _top 클래스 pose 수신 여부 (우선순위 추적용)
        self._tool_gripper_lock: threading.Lock = threading.Lock()

        # VS (slot 반납): 탑뷰 slot 위치 XY
        self._latest_slot_top: ToolPose = ToolPose()
        self._slot_top_lock: threading.Lock = threading.Lock()

        # setup()에서 생성
        self._movel_cli = None
        self._movej_cli = None
        self._stop_cli = None
        self._set_tcp_cli = None
        self._create_tcp_cli = None
        self._set_mode_cli = None
        self._gripper_cli = None
        self._get_posx_cli = None
        self._estop_sub = None
        self._handle_sub = None
        self._tool_gripper_sub = None
        self._slot_top_sub = None

        # E-4: config/toolbox.yaml vision_motion 섹션에서 좌표·범위 파라미터 로드
        self._load_toolbox_config()

    @property
    def estop_triggered(self) -> bool:
        return self._estop_triggered

    def reset_estop(self) -> None:
        """E-stop 래치 해제. ⚠️ 장수명 액션 서버는 E-stop 이후 새 goal을 수락하기
        전에 반드시 이 메서드를 호출해야 한다 — 그렇지 않으면 이후 모든
        run_sequence가 진입 즉시 estop 검사에서 중단된다(세션 단위 DoS)."""
        self._estop_triggered = False
        self._estop_event.clear()

    def request_stop(self) -> None:
        """외부(예: 액션 서버 ~/estop)에서 move_stop + 플래그 set."""
        if not self._estop_triggered:
            self._estop_triggered = True
            self._estop_event.set()
        if self._stop_cli is not None and self._stop_cli.service_is_ready():
            fut = self._stop_cli.call_async(MoveStop.Request())
            fut.add_done_callback(self._on_move_stop_result)
        else:
            self._node.get_logger().error('[engine] move_stop 서비스 미준비 — DSR 정지 명령 전송 불가 (수동 확인 필요)')

    def stop_motion(self) -> None:
        """시퀀스 실패 시 호출자가 사용 — blocking move_stop + 감속 대기.

        home_seq() 진입 전 move_stop을 먼저 호출해 타임아웃된 move_line이
        컨트롤러 큐에 남아 있는 경우 MoveJoint 충돌을 방지한다.
        (DB/PLC 기록은 호출자 책임 — 엔진은 DSR 정지만 담당)
        """
        if self._stop_cli is not None and self._stop_cli.service_is_ready():
            try:
                fut = self._stop_cli.call_async(MoveStop.Request())
                _done = threading.Event()
                fut.add_done_callback(lambda _: _done.set())
                _done.wait(timeout=0.2)
                self._node.get_logger().info('[engine] 시퀀스 실패 — DSR move_stop 전송')
                time.sleep(0.3)  # DSR 감속 완료 대기 — HIL 실측 후 조정
            except Exception as e:
                self._node.get_logger().error(f'[engine] move_stop 실패 (무시): {e}')
        else:
            self._node.get_logger().warn('[engine] move_stop 서비스 미준비 — DSR 정지 생략')

    # ── 설정 로딩 ─────────────────────────────────────────────────────────

    def _load_toolbox_config(self) -> None:
        """E-4: config/toolbox.yaml vision_motion 섹션에서 파라미터 로드.

        로드 실패 시 _config_valid=False 로 두어 run_sequence가 fail-closed로
        시퀀스를 거부한다 (S-5: 잘못된/누락된 workspace 한계 아래 모션 금지).
        """
        import yaml
        self._config_valid = False
        cfg_path = self._config_path
        try:
            with open(cfg_path, encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            vm = cfg.get('vision_motion', {})
            self._tool_approach_z_mm: float    = float(vm.get('tool_approach_z_mm', 234.0))
            self._tool_approach_ori: list      = list(vm.get('tool_approach_ori', [180.0, 180.0, 90.0]))
            self._tool_descent_ori: list       = list(vm.get('tool_descent_ori', [180.0, 180.0, 90.0]))
            self._gripper_scan_settle_sec: float = float(vm.get('gripper_scan_settle_sec', 2.0))
            lim = vm.get('workspace_limits', {})
            x = lim.get('x', [50.0, 800.0])
            y = lim.get('y', [-600.0, 600.0])
            z = lim.get('z', [-31.0, 700.0])
            self._vis_x_min, self._vis_x_max = float(x[0]), float(x[1])
            self._vis_y_min, self._vis_y_max = float(y[0]), float(y[1])
            self._vis_z_min, self._vis_z_max = float(z[0]), float(z[1])
            # 공구별 grasp_z_mm / return_z_mm 딕셔너리 로드
            self._grasp_z_map: dict[str, float] = {
                t['tool_id']: float(t['grasp_z_mm'])
                for t in cfg.get('tools', [])
                if 'grasp_z_mm' in t
            }
            self._return_z_map: dict[str, float] = {
                t['tool_id']: float(t['return_z_mm'])
                for t in cfg.get('tools', [])
                if 'return_z_mm' in t
            }
            self._staging_pickup_z_map: dict[str, float] = {
                t['tool_id']: float(t['staging_pickup_z_mm'])
                for t in cfg.get('tools', [])
                if 'staging_pickup_z_mm' in t
            }
            self._staging_place_z_map: dict[str, float] = {
                t['tool_id']: float(t['staging_place_z_mm'])
                for t in cfg.get('tools', [])
                if 'staging_place_z_mm' in t
            }
            # 공구별 grip_stroke (미등록 시 PULSE_GRIP_TOOL=650 사용)
            self._grip_stroke_map: dict[str, int] = {
                t['tool_id']: int(t['grip_stroke'])
                for t in cfg.get('tools', [])
                if 'grip_stroke' in t
            }
            # 공구별 slot XY (grasp_pose_base, m → mm 변환)
            self._slot_xy_map: dict[str, tuple[float, float]] = {
                t['tool_id']: (
                    float(t['grasp_pose_base']['x']) * 1000.0,
                    float(t['grasp_pose_base']['y']) * 1000.0,
                )
                for t in cfg.get('tools', [])
                if 'grasp_pose_base' in t
            }
            # 그리퍼 캠 스캔 자세 (vision_fetch_seq ③ / vision_return_seq ③)
            gc = vm.get('gripper_cam_scan', {})
            self._fetch_scan_j_deg: list = list(gc.get('fetch_j_deg',
                [-30.1, 15.5, 74.7, 20.9, 101.2, -27.8]))
            self._return_scan_j_deg: list = list(gc.get('return_j_deg',
                [-24.60, 32.49, 50.78, 22.42, 105.63, -19.92]))
            self._node.get_logger().info(
                f'[engine] toolbox.yaml 로드 완료 — '
                f'approach_z={self._tool_approach_z_mm}mm '
                f'ws_x=[{self._vis_x_min},{self._vis_x_max}] '
                f'grasp_z_map={self._grasp_z_map} '
                f'return_z_map={self._return_z_map} '
                f'slot_xy_map={self._slot_xy_map} '
                f'fetch_scan={self._fetch_scan_j_deg} '
                f'return_scan={self._return_scan_j_deg}'
            )
            self._config_valid = True
        except Exception as e:
            self._node.get_logger().error(f'[engine] toolbox.yaml 로드 실패: {e} — 기본값 사용')
            self._tool_approach_z_mm     = 234.0
            self._tool_approach_ori      = [180.0, 180.0, 90.0]
            self._tool_descent_ori       = [180.0, 180.0, 90.0]
            self._gripper_scan_settle_sec = 2.0
            self._vis_x_min, self._vis_x_max = 50.0, 800.0
            self._vis_y_min, self._vis_y_max = -600.0, 600.0
            self._vis_z_min, self._vis_z_max = -31.0, 700.0
            self._grasp_z_map = {}
            self._return_z_map = {}
            self._staging_pickup_z_map = {}
            self._staging_place_z_map = {}
            self._grip_stroke_map = {}
            self._slot_xy_map = {}
            self._fetch_scan_j_deg  = [-30.1, 15.5, 74.7, 20.9, 101.2, -27.8]
            self._return_scan_j_deg = [-24.60, 32.49, 50.78, 22.42, 105.63, -19.92]

    # ── setup: 클라이언트/구독 생성 + 필수 서비스 대기 ──────────────────────

    def setup(self, wait_timeout_sec: float = 10.0) -> None:
        """DSR 서비스 클라이언트·비전 구독·E-stop 구독 생성 + 필수 서비스 대기.

        필수 서비스(move_line/move_joint/set_current_tcp/gripper)가 없으면 RuntimeError.
        """
        p = f'/{self._robot_ns}'
        self._movel_cli      = self._node.create_client(MoveLine,           f'{p}/motion/move_line',         callback_group=self._cb_group)
        self._movej_cli      = self._node.create_client(MoveJoint,          f'{p}/motion/move_joint',        callback_group=self._cb_group)
        self._stop_cli       = self._node.create_client(MoveStop,           f'{p}/motion/move_stop',         callback_group=self._cb_group)
        self._set_tcp_cli    = self._node.create_client(SetCurrentTcp,      f'{p}/tcp/set_current_tcp',      callback_group=self._cb_group)
        self._create_tcp_cli = self._node.create_client(ConfigCreateTcp,    f'{p}/tcp/config_create_tcp',    callback_group=self._cb_group)
        self._set_mode_cli   = self._node.create_client(SetRobotMode,       f'{p}/system/set_robot_mode',    callback_group=self._cb_group)
        self._gripper_cli    = self._node.create_client(GripperSetPosition, '/gripper/set_position',       callback_group=self._cb_group)
        # VS: 현재 EE 포즈 조회 (DSR BASE 좌표계)
        self._get_posx_cli   = self._node.create_client(GetCurrentPosx,     f'{p}/aux_control/get_current_posx', callback_group=self._cb_group)

        self._node.get_logger().info('[engine] 서비스 대기 중...')
        for cli, name in [
            (self._movel_cli,   'move_line'),
            (self._movej_cli,   'move_joint'),
            (self._set_tcp_cli, 'set_current_tcp'),
            (self._gripper_cli, 'gripper/set_position'),
        ]:
            if not cli.wait_for_service(timeout_sec=wait_timeout_sec):
                self._node.get_logger().error(f'[engine] {name} 없음 — bringup 먼저 실행')
                raise RuntimeError(f'{name} 서비스 없음')
            self._node.get_logger().info(f'[engine] {name} 연결됨')

        # S-3: E-stop 구독 — /plc/e_stop (interfaces.md §4, Reliable + Transient Local)
        self._estop_sub = self._node.create_subscription(
            Bool, '/plc/e_stop', self._on_estop, _LATCHED_QOS,
            callback_group=self._cb_group,
        )

        # VS (서랍 손잡이): /vision/handle_pose 구독
        self._handle_sub = self._node.create_subscription(
            PointStamped, '/vision/handle_pose', self._on_handle_pose,
            qos_profile_sensor_data, callback_group=self._cb_group,
        )

        # 그리퍼 캠 공구 좌표 (XY + rz) — fetch/return 공통 토픽
        self._tool_gripper_sub = self._node.create_subscription(
            PoseStamped, '/vision/tool_gripper_pose', self._on_tool_gripper_pose,
            qos_profile_sensor_data, callback_group=self._cb_group,
        )

        # VS (slot 반납): slot rough XY (⑨번 이동용)
        self._slot_top_sub = self._node.create_subscription(
            PointStamped, '/vision/slot_top_pose', self._on_slot_top_pose,
            qos_profile_sensor_data, callback_group=self._cb_group,
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _on_estop(self, msg: Bool) -> None:
        """S-3: E-stop 수신 — 플래그 세팅 + Event set + DSR move_stop 즉시 요청."""
        if msg.data and not self._estop_triggered:
            self._estop_triggered = True
            self._estop_event.set()   # VS 루프에 즉시 전달
            self._node.get_logger().error('[engine] E-stop 수신 — 모션 즉시 중단 요청')
            if self._stop_cli.service_is_ready():
                fut = self._stop_cli.call_async(MoveStop.Request())
                fut.add_done_callback(self._on_move_stop_result)
            else:
                self._node.get_logger().error('[engine] move_stop 서비스 미준비 — DSR 정지 명령 전송 불가 (수동 확인 필요)')

    def _on_move_stop_result(self, future) -> None:
        """S-3: move_stop 응답 확인 — 실패 시 경고 로그."""
        try:
            res = future.result()
            if res is None or not res.success:
                self._node.get_logger().error('[engine] move_stop 응답 실패 — DSR 정지 미확인 (수동 확인 필요)')
            else:
                self._node.get_logger().info('[engine] move_stop 응답 확인')
        except Exception as e:
            self._node.get_logger().error(f'[engine] move_stop 응답 처리 오류: {e}')

    def _on_handle_pose(self, msg: PointStamped) -> None:
        """VS: 손잡이 중심 좌표 수신 (robot base frame, m → mm 변환)."""
        with self._handle_lock:
            self._latest_handle = HandlePose(
                x=msg.point.x * 1000.0,   # m → mm
                z=msg.point.z * 1000.0,
                valid=True,
            )

    def _on_tool_gripper_pose(self, msg: PoseStamped) -> None:
        """그리퍼 캠 공구 좌표 수신 (/vision/tool_gripper_pose) — fetch/return 공통."""
        # frame_id = "tool:{class_id}" 형식에서 감지된 클래스 추출 후 필터링.
        # socket_19mm_top 처럼 _top/_side 접미사 변형도 tool_id 기준으로 허용.
        frame_id = msg.header.frame_id
        is_top = False
        if frame_id.startswith("tool:"):
            detected_class = frame_id[len("tool:"):]
            current_tool = self._tool_id
            if current_tool and not detected_class.startswith(current_tool):
                return
            is_top = detected_class.endswith("_top")
            # _top pose가 이미 캐시에 있으면 side view는 무시
            if not is_top and self._latest_gripper_is_top:
                return
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        pca_theta = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        # PCA theta(공구 장축) → 로봇 rz(그리퍼 파지 방향) 변환: -90° 오프셋
        # ±180°가 동일 자세이므로 (-180°, 180°] 범위로 정규화
        rz = pca_theta - 90.0
        if rz < -180.0:
            rz += 360.0
        with self._tool_gripper_lock:
            self._latest_tool_gripper = ToolPose(
                x=msg.pose.position.x * 1000.0,
                y=msg.pose.position.y * 1000.0,
                z=msg.pose.position.z * 1000.0,
                rz=rz,
                valid=True,
            )
            self._latest_gripper_is_top = is_top

    def _on_slot_top_pose(self, msg: PointStamped) -> None:
        """탑뷰 D455f slot 위치 수신 — ⑧ MOVE_L_SLOT_XY 에서 rough XY 로 사용."""
        with self._slot_top_lock:
            self._latest_slot_top = ToolPose(
                x=msg.point.x * 1000.0,
                y=msg.point.y * 1000.0,
                z=msg.point.z * 1000.0,
                valid=True,
            )

    # ── 시퀀스 실행 ───────────────────────────────────────────────────────

    def run_sequence(
        self,
        steps: list,
        *,
        tool_id: str = "",
        cancel_check: Optional[Callable[[], bool]] = None,
        feedback_cb: Optional[Callable[[str, float], None]] = None,
    ) -> bool:
        """시퀀스 실행. tool_id는 현재 값으로 저장되어 _exec_*가 사용한다.

        - step별 cancel_check()(있으면) True 시 중단.
        - feedback_cb(step.kind.name, progress)(있으면) step 진입 시 호출.
        - E-stop 플래그도 계속 검사(S-3).
        """
        if not self._config_valid:
            self._node.get_logger().error(
                '[engine] toolbox.yaml 설정 무효 — 시퀀스 실행 거부 (S-5 fail-closed)'
            )
            return False
        self._tool_id = tool_id
        total = len(steps)
        for i, step in enumerate(steps):
            # S-3: E-stop 수신 시 다음 스텝 진입 전 중단
            if self._estop_triggered:
                self._node.get_logger().error(f'  E-stop — step {i+1}/{total} 진입 전 중단')
                return False
            # 외부 취소(예: 액션 서버 cancel) — 다음 스텝 진입 전 중단
            if cancel_check is not None and cancel_check():
                self._node.get_logger().error(f'  cancel — step {i+1}/{total} 진입 전 중단')
                return False
            self._node.get_logger().info(f'  step {i+1}/{total}: {step.kind.name}')
            progress = (i + 1) / total if total else 1.0
            if feedback_cb is not None:
                feedback_cb(step.kind.name, progress)
            ok = self._exec_step(step)
            if not ok:
                self._node.get_logger().error(f'  step {i+1} 실패 — 중단')
                return False
            # 물리적 집기/놓기 시점 알림 — step.marker("pick"/"place") 설정된 경우만
            # progress는 이미 계산된 값 재사용. 실행 성공 후 발행해야 DB 전이 타이밍 보장.
            if step.marker and feedback_cb is not None:
                feedback_cb(step.marker, progress)
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
            return self._exec_wait_vision_gripper_xy('FETCH')
        elif step.kind == StepKind.WAIT_VISION_RETURN_XY:
            return self._exec_wait_vision_gripper_xy('RETURN')
        elif step.kind == StepKind.MOVE_L_SLOT_XYZ:
            return self._exec_move_l_slot_xyz_return()
        elif step.kind == StepKind.MOVE_L_STAGING_XYZ:
            return self._exec_move_l_staging_xyz_return()
        elif step.kind == StepKind.MOVE_L_SLOT_XYZ_FETCH:
            return self._exec_move_l_slot_xyz_fetch()
        elif step.kind == StepKind.MOVE_L_STAGING_PLACE:
            return self._exec_move_l_staging_place()
        self._node.get_logger().warn(f'  알 수 없는 StepKind: {step.kind}')
        return False

    # ── TCP 설정 ──────────────────────────────────────────────────────────

    def _switch_robot_mode(self, mode: int) -> bool:
        """DSR 로봇 모드 전환. 0=MANUAL, 1=AUTONOMOUS."""
        if self._set_mode_cli is None or not self._set_mode_cli.service_is_ready():
            self._node.get_logger().warn(f'[engine] set_robot_mode 서비스 미준비 — 건너뜀 (target={mode})')
            return False
        req = SetRobotMode.Request()
        req.robot_mode = mode
        fut = self._set_mode_cli.call_async(req)
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        _done.wait(timeout=3.0)
        res = fut.result()
        if res is None or not res.success:
            self._node.get_logger().warn(f'[engine] set_robot_mode 실패 (target={mode})')
            return False
        mode_name = {0: 'MANUAL', 1: 'AUTONOMOUS'}.get(mode, str(mode))
        self._node.get_logger().info(f'[engine] 로봇 모드: {mode_name}')
        return True

    def set_tcp(self) -> bool:
        name = self._tcp_name
        # config_create_tcp / set_current_tcp 는 MANUAL 모드에서만 동작
        self._switch_robot_mode(0)
        try:
            if self._create_tcp_cli.service_is_ready():
                create_req = ConfigCreateTcp.Request()
                create_req.name = name
                create_req.pos  = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
                fut = self._create_tcp_cli.call_async(create_req)
                _done = threading.Event()
                fut.add_done_callback(lambda _: _done.set())
                _done.wait(timeout=5.0)
                res = fut.result()
                if res is None or not getattr(res, 'success', True):
                    # 이미 등록된 경우 실패 반환 — set_current_tcp 로 활성화 계속 시도
                    self._node.get_logger().warn(f'[engine] TCP 등록 실패 (이미 등록됐을 수 있음): {name}')
                else:
                    self._node.get_logger().info(f'[engine] TCP 등록 완료: {name} pos=[0,0,160,0,0,0]')
            else:
                self._node.get_logger().warn('[engine] config_create_tcp 서비스 미준비 — 건너뜀')

            # set_current_tcp는 시퀀스 실행의 전제 조건 — 미준비 시 중단
            if not self._set_tcp_cli.service_is_ready():
                self._node.get_logger().error('[engine] set_current_tcp 서비스 미준비 — 시퀀스 중단')
                return False

            set_req = SetCurrentTcp.Request()
            set_req.name = name
            fut = self._set_tcp_cli.call_async(set_req)
            _done = threading.Event()
            fut.add_done_callback(lambda _: _done.set())
            _done.wait(timeout=5.0)
            res = fut.result()
            if res is None or not getattr(res, 'success', True):
                msg = getattr(res, 'message', 'timeout')
                self._node.get_logger().error(f'[engine] set_current_tcp 응답 실패: name={name} msg={msg}')
                return False
            self._node.get_logger().info(f'[engine] TCP 활성화 완료: {name}')
            return True

        except Exception as e:
            self._node.get_logger().error(f'[engine] TCP 설정 예외: {e}')
            return False
        finally:
            self._switch_robot_mode(1)  # 모션 명령을 위해 AUTONOMOUS 복구

    # ── DSR 서비스 호출 ───────────────────────────────────────────────────

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
        _POLL = 0.1
        _TIMEOUT = 30.0
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        t_start = time.monotonic()
        while not _done.wait(timeout=_POLL):
            if self._estop_triggered:
                self._node.get_logger().error('  move_line E-stop 감지 — 이동 중단')
                return False
            if (time.monotonic() - t_start) >= _TIMEOUT:
                self._node.get_logger().error(f'  move_line timeout ({_TIMEOUT:.0f}s) — 서비스 무응답: pos={step.pose}')
                return False
        res = fut.result()
        if res is None:
            self._node.get_logger().error(f'  move_line timeout ({_TIMEOUT:.0f}s) — 서비스 무응답: pos={step.pose}')
            return False
        time.sleep(0.2)
        if not res.success:
            self._node.get_logger().error(f'  move_line 실패: pos={step.pose}')
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
        _POLL = 0.1
        _TIMEOUT = 20.0
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        t_start = time.monotonic()
        while not _done.wait(timeout=_POLL):
            if self._estop_triggered:
                self._node.get_logger().error('  move_joint E-stop 감지 — 이동 중단')
                return False
            if (time.monotonic() - t_start) >= _TIMEOUT:
                self._node.get_logger().error(f'  move_joint timeout ({_TIMEOUT:.0f}s) — 서비스 무응답: pos={step.pose}')
                return False
        res = fut.result()
        if res is None:
            self._node.get_logger().error(f'  move_joint timeout ({_TIMEOUT:.0f}s) — 서비스 무응답: pos={step.pose}')
            return False
        time.sleep(0.2)
        if not res.success:
            self._node.get_logger().error(f'  move_joint 실패: pos={step.pose}')
        return bool(res.success)

    def _grip(self, step: Step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0
        # tool_id별 grip_stroke 오버라이드 (PULSE_GRIP_TOOL 계열 step에만 적용)
        if pulse == PULSE_GRIP_TOOL and self._tool_id in self._grip_stroke_map:
            pulse = self._grip_stroke_map[self._tool_id]
            self._node.get_logger().info(f'  [GRIP] tool_id={self._tool_id!r} grip_stroke 오버라이드 → {pulse}')
        current = 400 if pulse > 450 else 0  # grip: 400mA, open/release: gripper_node 기본값

        req = GripperSetPosition.Request()
        req.position    = pulse
        req.current     = current
        req.timeout_sec = 0.0

        fut = self._gripper_cli.call_async(req)
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        _done.wait(timeout=5.0)
        res = fut.result()
        if res is None or not res.success:
            msg = res.message if res else 'timeout'
            self._node.get_logger().error(f'  gripper set_position 실패: {msg}')
            return False
        self._node.get_logger().info(f'  gripper ok — pos={res.final_position} cur={res.final_current}')
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
            self._node.get_logger().error(f'  visual_servo.yaml 로드 실패: {e}')
            return False

        ctrl = HandleServoController(
            cfg=cfg,
            get_handle=self._get_latest_handle,
            get_ee_pose=self._get_ee_pose_mm,
            estop_event=self._estop_event,
        )

        _DT = 0.033  # 30 Hz
        self._node.get_logger().info('  [VS] 시작')

        while not ctrl.is_terminal():
            if self._estop_triggered:
                return False

            try:
                cmd = ctrl.tick()
            except RuntimeError as e:
                self._node.get_logger().error(f'  [VS] EE 위치 조회 실패 — 루프 중단: {e}')
                return False

            if cmd.stop:
                time.sleep(_DT)
                continue

            if cmd.vx != 0.0 or cmd.vz != 0.0:
                ok = self._movel_delta(cmd.vx, cmd.vz, _DT)
                if not ok:
                    self._node.get_logger().error('  [VS] movel_delta 실패')
                    return False

            time.sleep(_DT)

        if ctrl.state == ServoState.DONE:
            self._node.get_logger().info('  [VS] XZ 정렬 완료')
            return True

        self._node.get_logger().error(f'  [VS] 실패: {ctrl.state.name}')
        return False

    # ── 공구 접근 VS (vision_fetch) ───────────────────────────────────────────

    def _exec_wait_vision_gripper_xy(self, label: str, timeout_sec: float = 15.0) -> bool:
        """fetch/return ④: 캐시 초기화 후 EMA 안정화 대기 → /vision/tool_gripper_pose 신규 수신.

        스캔 자세 도달 직후 카메라 진동과 EMA 미수렴으로 인한 좌표 불안정을
        방지하기 위해 settle_sec 동안 프레임을 수집 후 캐시 리셋, 안정화된 다음
        프레임을 최종 좌표로 사용한다 (config/toolbox.yaml gripper_scan_settle_sec).
        """
        settle_sec = self._gripper_scan_settle_sec

        # 초기 캐시 클리어
        with self._tool_gripper_lock:
            self._latest_tool_gripper = ToolPose(valid=False)
            self._latest_gripper_is_top = False

        # EMA 안정화 대기 — 이 기간 동안 vision node가 프레임 누적, EMA 수렴
        if settle_sec > 0.0:
            self._node.get_logger().info(
                f'  [WAIT_{label}] EMA 안정화 대기 {settle_sec:.1f}s (gripper_scan_settle_sec)...'
            )
            time.sleep(settle_sec)
            # 안정화 구간 누적된 캐시 버리고 새 프레임 대기
            with self._tool_gripper_lock:
                self._latest_tool_gripper = ToolPose(valid=False)
                self._latest_gripper_is_top = False

        self._node.get_logger().info(f'  [WAIT_{label}] 그리퍼 캠 좌표 대기 중...')
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self._tool_gripper_lock:
                if self._latest_tool_gripper.valid:
                    p = self._latest_tool_gripper
                    # 이후 MOVE_L_TOP_XY / STAGING_XYZ가 같은 pose를 사용하도록 고정 캡처
                    self._staged_gripper_pose = ToolPose(**vars(p))
                    self._node.get_logger().info(
                        f'  [WAIT_{label}] 수신 완료 → ({p.x:.1f}, {p.y:.1f}) mm rz={p.rz:.1f}° (staged)'
                    )
                    return True
            time.sleep(0.05)
        self._node.get_logger().error(
            f'  [WAIT_{label}] 타임아웃 ({timeout_sec:.0f}s) — /vision/tool_gripper_pose 확인 필요'
        )
        return False

    def _exec_move_l_top_xy(self) -> bool:
        """fetch ⑤⑧ / return ⑤⑧: 그리퍼 캠 XY + rz + approach_z 로 이동."""
        # WAIT_VISION에서 캡처한 고정 pose 사용 — 새 프레임으로 rz가 덮어써지는 것 방지
        pose = self._staged_gripper_pose

        if not pose.valid:
            self._node.get_logger().error('  [TOP_XY] 그리퍼 캠 좌표 미수신 (/vision/tool_gripper_pose)')
            return False
        if not math.isfinite(pose.rz) or not (-185.0 <= pose.rz <= 185.0):
            self._node.get_logger().error(f'  [TOP_XY] rz 비정상값 거부: {pose.rz!r}°')
            return False
        if not self._check_vision_coords('TOP_XY', pose.x, pose.y, self._tool_approach_z_mm):
            return False
        ori = list(self._tool_approach_ori)
        ori[2] = pose.rz
        pos = [pose.x, pose.y, self._tool_approach_z_mm] + ori
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(
            f'  [TOP_XY] → ({pose.x:.1f}, {pose.y:.1f}, {self._tool_approach_z_mm:.1f}) rz={pose.rz:.1f}°'
        )
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
            self._node.get_logger().error(f'  [VS_XY] visual_servo.yaml 로드 실패: {e}')
            return False

        ctrl = ToolServoController(
            cfg=cfg,
            get_tool=self._get_latest_gripper_tool,
            get_ee_pose=self._get_ee_pose_mm,
            estop_event=self._estop_event,
        )

        _DT = 0.033  # 30 Hz
        self._node.get_logger().info('  [VS_XY] 시작')

        while not ctrl.is_terminal():
            if self._estop_triggered:
                return False

            try:
                cmd = ctrl.tick()
            except RuntimeError as e:
                self._node.get_logger().error(f'  [VS_XY] EE 위치 조회 실패 — 루프 중단: {e}')
                return False

            if cmd.stop:
                time.sleep(_DT)
                continue

            if cmd.vx != 0.0 or cmd.vy != 0.0:
                ok = self._movel_delta_xy(cmd.vx, cmd.vy, _DT)
                if not ok:
                    self._node.get_logger().error('  [VS_XY] movel_delta_xy 실패')
                    return False

            time.sleep(_DT)

        if ctrl.state == ServoState.DONE:
            self._node.get_logger().info('  [VS_XY] XY 정렬 완료')
            return True

        self._node.get_logger().error(f'  [VS_XY] 실패: {ctrl.state.name}')
        return False

    def _exec_move_l_tool_xyz(self) -> bool:
        """fetch ⑥: 그리퍼 캠 XY + rz + grasp_z_mm 로 공구 위치까지 하강."""
        with self._tool_gripper_lock:
            pose = ToolPose(**vars(self._latest_tool_gripper))
        if not pose.valid:
            self._node.get_logger().error('  [TOOL_XYZ] 그리퍼 캠 좌표 미수신 (/vision/tool_gripper_pose)')
            return False

        grasp_z = self._grasp_z_map.get(self._tool_id)
        if grasp_z is not None:
            z = grasp_z
            self._node.get_logger().info(f'  [TOOL_XYZ] Z = {z:.2f}mm (toolbox.yaml, tool_id={self._tool_id})')
        else:
            z = pose.z
            self._node.get_logger().warn(
                f'  [TOOL_XYZ] tool_id={self._tool_id!r} grasp_z_mm 미등록 — 그리퍼 캠 Z 사용: {z:.2f}mm'
            )

        if not math.isfinite(pose.rz) or not (-185.0 <= pose.rz <= 185.0):
            self._node.get_logger().error(f'  [TOOL_XYZ] rz 비정상값 거부: {pose.rz!r}°')
            return False
        if not self._check_vision_coords('TOOL_XYZ', pose.x, pose.y, z):
            return False
        ori = list(self._tool_descent_ori)
        ori[2] = pose.rz
        pos = [pose.x, pose.y, z] + ori
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(f'  [TOOL_XYZ] → ({pose.x:.1f}, {pose.y:.1f}, {z:.1f}) rz={pose.rz:.1f}°')
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_staging_xyz_return(self) -> bool:
        """return ⑥: 그리퍼 캠 XY + rz + staging_pickup_z_mm 로 staging 파지 하강."""
        # WAIT_VISION에서 캡처한 고정 pose 사용 — TOP_XY와 동일 rz 보장
        pose = self._staged_gripper_pose
        if not pose.valid:
            self._node.get_logger().error('  [STAGING_XYZ] 그리퍼 캠 좌표 미수신 (/vision/tool_gripper_pose)')
            return False

        if not math.isfinite(pose.rz) or not (-185.0 <= pose.rz <= 185.0):
            self._node.get_logger().error(f'  [STAGING_XYZ] rz 비정상값 거부: {pose.rz!r}°')
            return False

        staging_z = self._staging_pickup_z_map.get(self._tool_id)
        if staging_z is None:
            self._node.get_logger().error(
                f'  [STAGING_XYZ] tool_id={self._tool_id!r} staging_pickup_z_mm 미등록 — 실행 중단'
            )
            return False
        z = staging_z
        self._node.get_logger().info(f'  [STAGING_XYZ] Z = {z:.2f}mm (staging_pickup_z_mm, tool_id={self._tool_id})')

        if not self._check_vision_coords('STAGING_XYZ', pose.x, pose.y, z):
            return False
        ori = list(self._tool_descent_ori)
        ori[2] = pose.rz  # RZ = vision rz (RX/RY는 config tool_descent_ori 사용)
        pos = [pose.x, pose.y, z] + ori
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(f'  [STAGING_XYZ] → ({pose.x:.1f}, {pose.y:.1f}, {z:.1f}) rz={pose.rz:.1f}°')
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_slot_xyz_return(self) -> bool:
        """return ⑩: toolbox.yaml slot XY + return_z_mm 로 slot 반납 하강."""
        slot = self._slot_xy_map.get(self._tool_id)
        if slot is None:
            self._node.get_logger().error(f'  [SLOT_XYZ] tool_id={self._tool_id!r} slot XY 미등록 (toolbox.yaml grasp_pose_base 확인)')
            return False
        x, y = slot

        return_z = self._return_z_map.get(self._tool_id)
        if return_z is not None:
            z = return_z
            self._node.get_logger().info(f'  [SLOT_XYZ] Z = {z:.2f}mm (toolbox.yaml, tool_id={self._tool_id})')
        else:
            self._node.get_logger().error(f'  [SLOT_XYZ] tool_id={self._tool_id!r} return_z_mm 미등록')
            return False

        if not self._check_vision_coords('SLOT_XYZ', x, y, z):
            return False
        pos = [x, y, z] + list(self._tool_descent_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(f'  [SLOT_XYZ] → ({x:.1f}, {y:.1f}, {z:.1f}) mm  tool_id={self._tool_id}')
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_slot_xyz_fetch(self) -> bool:
        """fetch ⑤: toolbox.yaml slot XY + grasp_z_mm 로 공구 파지 하강 (비전-free)."""
        slot = self._slot_xy_map.get(self._tool_id)
        if slot is None:
            self._node.get_logger().error(
                f'  [SLOT_XYZ_FETCH] tool_id={self._tool_id!r} slot XY 미등록 (toolbox.yaml grasp_pose_base 확인)'
            )
            return False
        x, y = slot
        grasp_z = self._grasp_z_map.get(self._tool_id)
        if grasp_z is None:
            self._node.get_logger().error(
                f'  [SLOT_XYZ_FETCH] tool_id={self._tool_id!r} grasp_z_mm 미등록'
            )
            return False
        z = grasp_z
        if not self._check_vision_coords('SLOT_XYZ_FETCH', x, y, z):
            return False
        pos = [x, y, z] + list(self._tool_descent_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(
            f'  [SLOT_XYZ_FETCH] → ({x:.1f}, {y:.1f}, {z:.1f}) mm  tool_id={self._tool_id}'
        )
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_staging_place(self) -> bool:
        """fetch ⑨: SOCKET_BOTTOM XY/ori + per-tool staging_place_z_mm 로 staging 하강."""
        from unit_actions.toolbox_motion import SOCKET_BOTTOM
        place_z = self._staging_place_z_map.get(self._tool_id)
        if place_z is None:
            self._node.get_logger().warn(
                f'  [STAGING_PLACE] tool_id={self._tool_id!r} staging_place_z_mm 미등록 — SOCKET_BOTTOM z 사용'
            )
            place_z = SOCKET_BOTTOM[2]
        x, y = SOCKET_BOTTOM[0], SOCKET_BOTTOM[1]
        ori = list(SOCKET_BOTTOM[3:])
        pos = [x, y, place_z] + ori
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(
            f'  [STAGING_PLACE] → ({x:.1f}, {y:.1f}, {place_z:.2f}) mm  tool_id={self._tool_id}'
        )
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_slot_xy(self) -> bool:
        """return ⑨⑫: toolbox.yaml grasp_pose_base XY + approach_z 로 slot 위 이동."""
        slot = self._slot_xy_map.get(self._tool_id)
        if slot is None:
            self._node.get_logger().error(f'  [SLOT_XY] tool_id={self._tool_id!r} slot XY 미등록 (toolbox.yaml grasp_pose_base 확인)')
            return False
        x, y = slot
        if not self._check_vision_coords('SLOT_XY', x, y, self._tool_approach_z_mm):
            return False
        pos = [x, y, self._tool_approach_z_mm] + list(self._tool_approach_ori)
        step = Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=self._vel_l, acc=self._acc_l)
        self._node.get_logger().info(f'  [SLOT_XY] → ({x:.1f}, {y:.1f}, {self._tool_approach_z_mm:.1f}) mm  tool_id={self._tool_id}')
        return self._movel(step, DR_MV_MOD_ABS)

    # ── 좌표 검증 ─────────────────────────────────────────────────────────

    def _check_vision_coords(self, label: str, x: float, y: float, z: float) -> bool:
        if x == 0.0 and y == 0.0 and z == 0.0:
            self._node.get_logger().error(f'[engine] {label} 좌표 미설정 (0,0,0) — 실행 거부')
            return False
        for val, lo, hi, axis in [
            (x, self._vis_x_min, self._vis_x_max, 'x'),
            (y, self._vis_y_min, self._vis_y_max, 'y'),
            (z, self._vis_z_min, self._vis_z_max, 'z'),
        ]:
            if not (lo <= val <= hi):
                self._node.get_logger().error(
                    f'[engine] {label} {axis}={val:.1f}mm 범위 초과 [{lo}, {hi}] — 실행 거부'
                )
                return False
        return True

    # ── VS 헬퍼 ───────────────────────────────────────────────────────────

    def _get_latest_gripper_tool(self) -> ToolPose:
        with self._tool_gripper_lock:
            return ToolPose(
                x=self._latest_tool_gripper.x,
                y=self._latest_tool_gripper.y,
                z=self._latest_tool_gripper.z,
                valid=self._latest_tool_gripper.valid,
            )

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
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        _done.wait(timeout=2.0)
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
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        _done.wait(timeout=0.45)
        res = fut.result()
        return res is not None and res.success

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
        _done = threading.Event()
        fut.add_done_callback(lambda _: _done.set())
        _done.wait(timeout=0.45)
        res = fut.result()
        return res is not None and res.success
