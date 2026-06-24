"""home_on_start.py
──────────────────
bringup 직후 TCP/Tool 설정 후 모드별 동작:
  virtual: TCP + Tool 설정 → 홈 자세 이동
  real:    TCP + Tool 설정만 수행 (홈 이동 스킵 — 현재 자세 불명으로 충돌 위험)

설정값은 config/robot_poses.yaml에서 로드 (E-4).
dsr_controller2 spawner 종료 후 TimerAction으로 트리거된다.
완료 또는 실패 후 rclpy.shutdown()으로 종료.
"""

import os

import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveJoint, ConfigCreateTcp, SetCurrentTcp, ConfigCreateTool, SetCurrentTool, SetRobotMode

# DSR 네이티브 단위: degree (MoveJoint 서비스 직접 전달용)
JOINT_HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

_DEFAULT_TCP_NAME     = 'GripperDA_v1'
_DEFAULT_TCP_OFFSET   = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]

_DEFAULT_TOOL_NAME    = 'GripperDA_v1'
_DEFAULT_TOOL_WEIGHT  = 0.5
_DEFAULT_TOOL_COG     = [0.0, 0.0, 80.0]
# RH-P12-RN 원통 근사 (r=40mm, h=160mm, m=0.5kg) — [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] kg·m²
_DEFAULT_TOOL_INERTIA = [0.00127, 0.00127, 0.0004, 0.0, 0.0, 0.0]

_DEFAULT_VEL_J = 12.0   # config/robot_poses.yaml motion_limits 기본값과 일치
_DEFAULT_ACC_J = 20.0

# S-3/S-7: Transient Local QoS — retained 값 즉시 수신
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _load_robot_poses_yaml() -> dict:
    candidates = [
        os.environ.get('FINAL_PROJECT_ROOT', ''),
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'),
    ]
    for base in candidates:
        path = os.path.join(base, 'config', 'robot_poses.yaml')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
    return {}


def _load_tcp_config() -> tuple[str, list[float]]:
    data = _load_robot_poses_yaml()
    tcp = data.get('tcp', {})
    return tcp.get('name', _DEFAULT_TCP_NAME), tcp.get('offset_mm', _DEFAULT_TCP_OFFSET)


def _load_tool_config() -> tuple[str, float, list[float], list[float]]:
    data = _load_robot_poses_yaml()
    tool = data.get('tool', {})
    return (
        tool.get('name', _DEFAULT_TOOL_NAME),
        float(tool.get('weight_kg', _DEFAULT_TOOL_WEIGHT)),
        tool.get('cog_mm', _DEFAULT_TOOL_COG),
        tool.get('inertia_kgm2', _DEFAULT_TOOL_INERTIA),
    )


def _load_motion_limits() -> tuple[float, float]:
    data = _load_robot_poses_yaml()
    limits = data.get('motion_limits', {})
    return (
        float(limits.get('vel_j_deg_s', _DEFAULT_VEL_J)),
        float(limits.get('acc_j_deg_s2', _DEFAULT_ACC_J)),
    )


class HomeOnStart(Node):
    def __init__(self) -> None:
        super().__init__('home_on_start')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('mode', 'virtual')
        ns         = self.get_parameter('robot_ns').get_parameter_value().string_value
        self._mode = self.get_parameter('mode').get_parameter_value().string_value

        self._tcp_name, self._tcp_offset = _load_tcp_config()
        self._tool_name, self._tool_weight, self._tool_cog, self._tool_inertia = _load_tool_config()
        self._vel_j, self._acc_j = _load_motion_limits()

        # S-3: E-stop 플래그
        self._estop_triggered: bool = False

        self._cb = ReentrantCallbackGroup()
        p = f'/{ns}'

        self._set_mode_cli    = self.create_client(SetRobotMode,     f'{p}/system/set_robot_mode',  callback_group=self._cb)
        self._create_tcp_cli  = self.create_client(ConfigCreateTcp,  f'{p}/tcp/config_create_tcp',  callback_group=self._cb)
        self._set_tcp_cli     = self.create_client(SetCurrentTcp,    f'{p}/tcp/set_current_tcp',    callback_group=self._cb)
        self._create_tool_cli = self.create_client(ConfigCreateTool, f'{p}/tool/config_create_tool', callback_group=self._cb)
        self._set_tool_cli    = self.create_client(SetCurrentTool,   f'{p}/tool/set_current_tool',  callback_group=self._cb)
        self._movej_cli       = self.create_client(MoveJoint,        f'{p}/motion/move_joint',      callback_group=self._cb)

        # S-7: is_moving 발행 — Transient Local로 toolbox_seq_runner가 retained 값 수신
        self._is_moving_pub = self.create_publisher(Bool, '/motion/is_moving', _LATCHED_QOS)

        # S-3: E-stop 구독 — bringup 중 E-stop 수신 시 홈 이동 차단
        self._estop_sub = self.create_subscription(
            Bool, '/plc/e_stop', self._on_estop, _LATCHED_QOS, callback_group=self._cb
        )

        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb)
        self._done = False

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self._estop_triggered:
            self._estop_triggered = True
            self.get_logger().error('[home] E-stop 수신 — 홈 이동 중단')

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        if not self._setup_tcp():
            rclpy.shutdown()
            return

        if not self._setup_tool():
            rclpy.shutdown()
            return

        # S-3: E-stop 확인 후 홈 이동
        if self._mode == 'virtual':
            if self._estop_triggered:
                self.get_logger().error('[home] E-stop 수신 — 홈 이동 스킵')
            else:
                self._is_moving_pub.publish(Bool(data=True))   # S-7
                self._move_home()
                self._is_moving_pub.publish(Bool(data=False))  # S-7

        rclpy.shutdown()

    def _set_robot_mode(self, mode: int) -> bool:
        """DSR 로봇 모드 전환. 0=MANUAL, 1=AUTONOMOUS. TCP/Tool 설정은 MANUAL 필요."""
        if not self._set_mode_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'[home] set_robot_mode 서비스 미준비 — 건너뜀 (target={mode})')
            return False
        req = SetRobotMode.Request()
        req.robot_mode = mode
        fut = self._set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn(f'[home] set_robot_mode 실패 (target={mode})')
            return False
        mode_name = {0: 'MANUAL', 1: 'AUTONOMOUS'}.get(mode, str(mode))
        self.get_logger().info(f'[home] 로봇 모드: {mode_name}')
        return True

    def _setup_tcp(self) -> bool:
        # config_create_tcp / set_current_tcp 는 MANUAL 모드에서만 동작
        self._set_robot_mode(0)
        try:
            if self._create_tcp_cli.wait_for_service(timeout_sec=5.0):
                req = ConfigCreateTcp.Request()
                req.name = self._tcp_name
                req.pos  = [float(v) for v in self._tcp_offset]
                fut = self._create_tcp_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                res = fut.result()
                if res is None or not res.success:
                    # 이미 등록된 경우 실패 반환 — set_current_tcp 로 활성화 계속 시도
                    self.get_logger().warn(f'[home] TCP 등록 실패 (이미 등록됐을 수 있음): {self._tcp_name}')
                else:
                    self.get_logger().info(f'[home] TCP 등록: {self._tcp_name} offset={self._tcp_offset}')
            else:
                self.get_logger().warn('[home] config_create_tcp 서비스 미준비 — 건너뜀')

            if not self._set_tcp_cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('[home] set_current_tcp 서비스 없음 — TCP 활성화 실패')
                return False

            req = SetCurrentTcp.Request()
            req.name = self._tcp_name
            fut = self._set_tcp_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res is None or not res.success:
                self.get_logger().error(f'[home] TCP 활성화 실패: {self._tcp_name}')
                return False
            self.get_logger().info(f'[home] TCP 활성화: {self._tcp_name}')
            return True
        finally:
            self._set_robot_mode(1)  # 모션 명령을 위해 AUTONOMOUS 복구

    def _setup_tool(self) -> bool:
        # config_create_tool / set_current_tool 도 MANUAL 모드 필요
        self._set_robot_mode(0)
        try:
            if self._create_tool_cli.wait_for_service(timeout_sec=5.0):
                req = ConfigCreateTool.Request()
                req.name    = self._tool_name
                req.weight  = self._tool_weight
                req.cog     = [float(v) for v in self._tool_cog]
                req.inertia = [float(v) for v in self._tool_inertia]
                fut = self._create_tool_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                res = fut.result()
                if res is None or not res.success:
                    self.get_logger().warn(f'[home] Tool 등록 실패 (이미 등록됐을 수 있음): {self._tool_name}')
                else:
                    self.get_logger().info(
                        f'[home] Tool 등록: {self._tool_name} '
                        f'weight={self._tool_weight}kg cog={self._tool_cog}mm'
                    )
            else:
                self.get_logger().warn('[home] config_create_tool 서비스 미준비 — 건너뜀')

            if not self._set_tool_cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('[home] set_current_tool 서비스 없음 — payload 활성화 실패')
                return False

            req = SetCurrentTool.Request()
            req.name = self._tool_name
            fut = self._set_tool_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res is None or not res.success:
                self.get_logger().error(f'[home] Tool 활성화 실패: {self._tool_name}')
                return False
            self.get_logger().info(f'[home] Tool 활성화: {self._tool_name}')
            return True
        finally:
            self._set_robot_mode(1)  # AUTONOMOUS 복구

    def _move_home(self) -> None:
        if not self._movej_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('[home] move_joint 서비스 없음 — bringup 확인')
            return

        req = MoveJoint.Request()
        req.pos        = [float(v) for v in JOINT_HOME_DEG]
        req.vel        = self._vel_j
        req.acc        = self._acc_j
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0

        self.get_logger().info(f'[home] 홈 이동 중: {JOINT_HOME_DEG} vel={self._vel_j}deg/s')
        fut = self._movej_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)
        res = fut.result()
        if res and res.success:
            self.get_logger().info('[home] 홈 이동 완료')
        else:
            self.get_logger().error('[home] 홈 이동 실패 또는 응답 없음 (virtual 모드라면 정상)')


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = HomeOnStart()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
