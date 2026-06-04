"""toolbox_seq_runner.py
────────────────────────
toolbox_motion.py 시퀀스를 virtual/real 모드에서 실행하는 테스트 노드.

그리퍼는 action client(/gripper/grasp)를 사용해 파지 완료 확인 후 다음 스텝 진행.
gripper_node의 gripper_command_action_enabled: true 필요.
action 서버 미가동 시 fallback으로 topic publish + sleep 사용.

실행:
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=open_0
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=close_0
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=open_1
  ros2 run motion toolbox_seq_runner --ros-args -p sequence:=close_1

  sequence 옵션:
    open_0  / close_0  — layer 0 (1층 서랍)
    open_1  / close_1  — layer 1 (2층 서랍)

단위: toolbox_motion.py 좌표는 DSR 네이티브(mm/deg) → move_line/move_joint 직접 전달.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_msgs.msg import String
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp

try:
    from interfaces.action import Grasp as GripperCommand
    _GRIPPER_ACTION_AVAILABLE = True
except ImportError:
    _GRIPPER_ACTION_AVAILABLE = False
    GripperCommand = None

# unit_actions는 ros2_ws 밖에 있으므로 경로 추가
sys.path.insert(0, '/home/kimsungyeoun/Final_project')
from unit_actions.toolbox_motion import (
    StepKind,
    drawer_open_seq,
    drawer_close_seq,
)

DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1


class ToolboxSeqRunner(Node):

    def __init__(self) -> None:
        super().__init__('toolbox_seq_runner')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('sequence', 'open_0')
        self.declare_parameter('tcp_name', 'GripperDA_v1')

        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        seq_name = self.get_parameter('sequence').get_parameter_value().string_value
        self._tcp_name = self.get_parameter('tcp_name').get_parameter_value().string_value

        self._cb_group = ReentrantCallbackGroup()

        p = f'/{ns}'
        self._movel_cli      = self.create_client(MoveLine,        f'{p}/motion/move_line',       callback_group=self._cb_group)
        self._movej_cli      = self.create_client(MoveJoint,       f'{p}/motion/move_joint',      callback_group=self._cb_group)
        self._stop_cli       = self.create_client(MoveStop,        f'{p}/motion/move_stop',       callback_group=self._cb_group)
        self._set_tcp_cli    = self.create_client(SetCurrentTcp,   f'{p}/tcp/set_current_tcp',    callback_group=self._cb_group)
        self._create_tcp_cli = self.create_client(ConfigCreateTcp, f'{p}/tcp/config_create_tcp',  callback_group=self._cb_group)
        self._gripper_pub    = self.create_publisher(String, '/gripper/cmd_direct', 10)

        # 그리퍼 action client — 동기 파지 완료 확인용
        # wait_for_server는 init 때 한 번만 → spin 중에 재확인하면 서버를 잃어버림
        self._gripper_action_client = None
        self._gripper_action_ready = False
        if _GRIPPER_ACTION_AVAILABLE:
            self._gripper_action_client = ActionClient(
                self, GripperCommand, '/gripper/grasp', callback_group=self._cb_group
            )
            self.get_logger().info('[runner] 그리퍼 action 서버 대기 중...')
            if self._gripper_action_client.wait_for_server(timeout_sec=5.0):
                self._gripper_action_ready = True
                self.get_logger().info('[runner] 그리퍼 action 서버 연결됨: /gripper/grasp')
            else:
                self.get_logger().warn('[runner] 그리퍼 action 서버 없음 — topic fallback 사용')

        self.get_logger().info(f'[runner] 서비스 대기 중...')
        for cli, name in [
            (self._movel_cli,   'move_line'),
            (self._movej_cli,   'move_joint'),
            (self._set_tcp_cli, 'set_current_tcp'),
        ]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'[runner] {name} 없음 — bringup 먼저 실행')
                raise RuntimeError(f'{name} 서비스 없음')
            self.get_logger().info(f'[runner] {name} 연결됨')

        self._seq_name = seq_name
        self._timer = self.create_timer(0.5, self._run_once, callback_group=self._cb_group)
        self._done = False

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        seq = self._resolve_sequence(self._seq_name)
        if seq is None:
            self.get_logger().error(f'[runner] 알 수 없는 sequence: {self._seq_name}')
            self.get_logger().error('[runner] 사용 가능: open_0 close_0 open_1 close_1')
            return

        self._set_tcp(self._tcp_name)

        self.get_logger().info(f'[runner] 시퀀스 시작: {self._seq_name} ({len(seq)} steps)')
        ok = self._run_sequence(seq)
        if ok:
            self.get_logger().info(f'[runner] 시퀀스 완료: {self._seq_name}')
        else:
            self.get_logger().error(f'[runner] 시퀀스 실패: {self._seq_name}')
        import threading
        threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def _resolve_sequence(self, name: str):
        mapping = {
            'open_0':  lambda: drawer_open_seq(0),
            'close_0': lambda: drawer_close_seq(0),
            'open_1':  lambda: drawer_open_seq(1),
            'close_1': lambda: drawer_close_seq(1),
        }
        fn = mapping.get(name)
        return fn() if fn else None

    def _run_sequence(self, steps: list) -> bool:
        for i, step in enumerate(steps):
            self.get_logger().info(f'  step {i+1}/{len(steps)}: {step.kind.name}')
            ok = self._exec_step(step)
            if not ok:
                self.get_logger().error(f'  step {i+1} 실패 — 중단')
                return False
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
        self.get_logger().warn(f'  알 수 없는 StepKind: {step.kind}')
        return False

    def _set_tcp(self, name: str) -> bool:
        # chamjo 패턴: fire-and-forget (응답 대기 없이 발송 후 짧은 sleep)
        # Doosan 컨트롤러는 TCP 명령을 처리하면서도 success=False를 반환하는 경우가 있음
        try:
            # 1단계: 좌표로 TCP 등록 (이미 있으면 덮어쓰기)
            if self._create_tcp_cli.service_is_ready():
                create_req = ConfigCreateTcp.Request()
                create_req.name = name
                create_req.pos  = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
                self._create_tcp_cli.call_async(create_req)
                self.get_logger().info(f'[runner] TCP 등록 요청: {name} pos=[0,0,160,0,0,0]')
            else:
                self.get_logger().warn('[runner] config_create_tcp 서비스 미준비 — 건너뜀')

            # 2단계: 등록된 TCP를 활성화
            if self._set_tcp_cli.service_is_ready():
                set_req = SetCurrentTcp.Request()
                set_req.name = name
                self._set_tcp_cli.call_async(set_req)
                self.get_logger().info(f'[runner] TCP 활성화 요청: {name}')
            else:
                self.get_logger().warn('[runner] set_current_tcp 서비스 미준비 — 건너뜀')

            time.sleep(0.3)  # 컨트롤러가 TCP 설정 처리할 시간
        except Exception as e:
            self.get_logger().warn(f'[runner] TCP 설정 예외 (무시): {e}')

        return True  # 항상 시퀀스 진행 (chamjo 동일 패턴)

    def _movel(self, step, mode: int) -> bool:
        req = MoveLine.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = [step.vel or 250.0, step.vel or 76.5]
        req.acc        = [step.acc or 1000.0, step.acc or 306.0]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = mode
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movel_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f'  move_line 실패: pos={step.pose}')
        return ok

    def _movej(self, step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = step.vel or 60.0
        req.acc        = step.acc or 100.0
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movej_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f'  move_joint 실패: pos={step.pose}')
        return ok

    def _grip(self, step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0

        if pulse == 0:
            cmd = 'open'
            grasp_force = 0.0
        elif pulse <= 450:
            cmd = f'custom {pulse} 0'
            grasp_force = 0.0
        else:
            cmd = f'custom {pulse} 400'
            grasp_force = 400.0

        # action 서버가 살아있으면 동기 파지 완료 대기
        if self._gripper_action_ready:
            return self._grip_via_action(cmd, grasp_force)

        # fallback: topic publish + sleep
        self.get_logger().warn('  [grip] action 서버 없음 — topic fallback')
        msg = String()
        msg.data = cmd
        self._gripper_pub.publish(msg)
        self.get_logger().info(f'  gripper cmd(topic): {cmd}')
        time.sleep(1.0)
        return True

    def _grip_via_action(self, cmd: str, grasp_force: float) -> bool:
        """action goal 전송 후 result 수신까지 블로킹."""

        goal = GripperCommand.Goal()
        goal.tool_id           = 'drawer_handle'
        goal.approach_direction = cmd          # open / close / custom 정보 전달
        goal.grasp_force       = float(grasp_force)

        self.get_logger().info(f'  gripper action goal: {cmd} force={grasp_force}')

        done_event = threading.Event()
        result_holder: dict = {'success': False, 'message': ''}

        def _feedback_cb(fb):
            self.get_logger().debug(
                f'  gripper feedback: phase={fb.feedback.phase} progress={fb.feedback.progress:.2f}'
            )

        def _result_cb(future):
            try:
                res = future.result()
                result_holder['success'] = bool(res.result.success)
                result_holder['message'] = str(res.result.message)
            except Exception as e:
                result_holder['success'] = False
                result_holder['message'] = str(e)
            done_event.set()

        def _goal_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('  gripper action goal rejected')
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(_result_cb)

        send_future = self._gripper_action_client.send_goal_async(
            goal, feedback_callback=_feedback_cb
        )
        send_future.add_done_callback(_goal_cb)

        # action_max_wait_sec(기본 20s) + 여유 5초
        done_event.wait(timeout=25.0)

        ok = result_holder['success']
        msg_str = result_holder['message']
        if ok:
            self.get_logger().info(f'  gripper action 완료: {msg_str}')
        else:
            self.get_logger().error(f'  gripper action 실패: {msg_str}')
        return ok


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ToolboxSeqRunner()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except RuntimeError as e:
        print(f'[runner] 초기화 실패: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
