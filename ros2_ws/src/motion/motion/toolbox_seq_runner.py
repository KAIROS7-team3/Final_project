"""toolbox_seq_runner.py
────────────────────────
toolbox_motion.py 시퀀스를 virtual/real 모드에서 실행하는 테스트 노드.

chamjo robot_action_server_node.py의 _run_sequence/_movel/_movej 패턴 재사용.

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

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp

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
        elif pulse <= 450:
            # release 계열 (TW: gripper_release = stroke 450) — current 0으로 config 기본값 사용
            cmd = f'custom {pulse} 0'
        else:
            # grip 계열 (TW: gripper_grap_boxhand = stroke 600) — current 400mA
            cmd = f'custom {pulse} 400'
        msg = String()
        msg.data = cmd
        self._gripper_pub.publish(msg)
        self.get_logger().info(f'  gripper cmd: {cmd}')
        time.sleep(0.5)
        return True


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
