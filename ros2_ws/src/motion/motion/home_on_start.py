"""home_on_start.py
──────────────────
bringup 직후 로봇을 홈 자세 [0, 0, 90, 0, 90, 0]deg 로 이동시키는 일회성 노드.
dsr_controller2 spawner 종료 후 TimerAction으로 트리거된다.

완료 또는 실패 후 rclpy.shutdown()으로 종료.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveJoint

JOINT_HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
VEL_J  = 30.0   # deg/s — 낮게 설정해서 안전하게 이동
ACC_J  = 50.0


class HomeOnStart(Node):
    def __init__(self) -> None:
        super().__init__('home_on_start')

        self.declare_parameter('robot_ns', 'dsr01')
        ns = self.get_parameter('robot_ns').get_parameter_value().string_value

        self._cb = ReentrantCallbackGroup()
        self._cli = self.create_client(
            MoveJoint, f'/{ns}/motion/move_joint', callback_group=self._cb
        )
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb)
        self._done = False

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        if not self._cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('[home] move_joint 서비스 없음 — bringup 확인')
            return

        req = MoveJoint.Request()
        req.pos        = [float(v) for v in JOINT_HOME_DEG]
        req.vel        = VEL_J
        req.acc        = ACC_J
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0   # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0

        self.get_logger().info(f'[home] 홈 이동 중: {JOINT_HOME_DEG}')
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)
        res = fut.result()
        if res and res.success:
            self.get_logger().info('[home] 홈 이동 완료')
        else:
            self.get_logger().warn('[home] 홈 이동 응답 없음 (virtual 에뮬레이터 정상)')


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
