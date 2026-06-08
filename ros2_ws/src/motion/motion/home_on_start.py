"""home_on_start.py
──────────────────
bringup 직후 TCP 설정 후 모드별 동작:
  virtual: TCP 설정 → 홈 자세 [0, 0, 90, 0, 90, 0]deg 이동
  real:    TCP 설정만 수행 (홈 이동 스킵 — 현재 자세 불명으로 충돌 위험)

TCP 설정값은 config/robot_poses.yaml tcp 섹션에서 로드 (E-4).
dsr_controller2 spawner 종료 후 TimerAction으로 트리거된다.
완료 또는 실패 후 rclpy.shutdown()으로 종료.
"""

import os
from typing import Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveJoint, ConfigCreateTcp, SetCurrentTcp

JOINT_HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
VEL_J  = 30.0
ACC_J  = 50.0

_DEFAULT_TCP_NAME   = 'GripperDA_v1'
_DEFAULT_TCP_OFFSET = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]


def _load_tcp_config() -> tuple[str, list[float]]:
    candidates = [
        os.environ.get('FINAL_PROJECT_ROOT', ''),
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'),
    ]
    for base in candidates:
        path = os.path.join(base, 'config', 'robot_poses.yaml')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f)
            tcp = data.get('tcp', {})
            return tcp.get('name', _DEFAULT_TCP_NAME), tcp.get('offset_mm', _DEFAULT_TCP_OFFSET)
    return _DEFAULT_TCP_NAME, _DEFAULT_TCP_OFFSET


class HomeOnStart(Node):
    def __init__(self) -> None:
        super().__init__('home_on_start')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('mode', 'virtual')
        ns        = self.get_parameter('robot_ns').get_parameter_value().string_value
        self._mode = self.get_parameter('mode').get_parameter_value().string_value

        self._tcp_name, self._tcp_offset = _load_tcp_config()

        self._cb = ReentrantCallbackGroup()
        p = f'/{ns}'
        self._create_tcp_cli = self.create_client(
            ConfigCreateTcp, f'{p}/tcp/config_create_tcp', callback_group=self._cb
        )
        self._set_tcp_cli = self.create_client(
            SetCurrentTcp, f'{p}/tcp/set_current_tcp', callback_group=self._cb
        )
        self._movej_cli = self.create_client(
            MoveJoint, f'{p}/motion/move_joint', callback_group=self._cb
        )
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb)
        self._done = False

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        self._setup_tcp()

        if self._mode == 'virtual':
            self._move_home()

        rclpy.shutdown()

    def _setup_tcp(self) -> None:
        if self._create_tcp_cli.wait_for_service(timeout_sec=5.0):
            req = ConfigCreateTcp.Request()
            req.name = self._tcp_name
            req.pos  = [float(v) for v in self._tcp_offset]
            fut = self._create_tcp_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            self.get_logger().info(f'[home] TCP 등록: {self._tcp_name} offset={self._tcp_offset}')
        else:
            self.get_logger().warn('[home] config_create_tcp 서비스 미준비 — 건너뜀')

        if not self._set_tcp_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('[home] set_current_tcp 서비스 없음 — TCP 활성화 실패')
            return

        req = SetCurrentTcp.Request()
        req.name = self._tcp_name
        fut = self._set_tcp_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info(f'[home] TCP 활성화: {self._tcp_name}')

    def _move_home(self) -> None:
        if not self._movej_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('[home] move_joint 서비스 없음 — bringup 확인')
            return

        req = MoveJoint.Request()
        req.pos        = [float(v) for v in JOINT_HOME_DEG]
        req.vel        = VEL_J
        req.acc        = ACC_J
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0

        self.get_logger().info(f'[home] 홈 이동 중: {JOINT_HOME_DEG}')
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
