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

import logging
import os
import sys
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
from dsr_msgs2.srv import SetCurrentTcp, ConfigCreateTcp
from interfaces.srv import GripperSetPosition


def _add_unit_actions_to_path() -> None:
    """unit_actions/ 는 ros2_ws 밖(레포 루트)에 있고 ROS2 패키지가 아니라
    colcon이 설치하지 않으므로, 소스 트리의 레포 루트를 sys.path에 추가한다.

    탐색 순서:
      1. FINAL_PROJECT_ROOT 환경변수 (팀원별 체크아웃 경로 — 권장)
      2. 이 파일 기준 상위 디렉토리에서 unit_actions/ 를 가진 루트 탐색
         (소스 트리 직접 실행 또는 colcon --symlink-install 시 동작)
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
)

DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# 비전 좌표 허용 범위 (DSR BASE 좌표계, mm)
# 실제 비전 좌표 수신 후 config/toolbox.yaml 이관 예정 (E-4)
_VIS_X_MIN, _VIS_X_MAX = 50.0,  800.0
_VIS_Y_MIN, _VIS_Y_MAX = -600.0, 600.0
_VIS_Z_MIN, _VIS_Z_MAX = 0.0,    700.0


class ToolboxSeqRunner(Node):

    def __init__(self) -> None:
        super().__init__('toolbox_seq_runner')

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('sequence', 'open_0')
        self.declare_parameter('tcp_name', 'GripperDA_v1')
        self.declare_parameter('vision_x_mm', 0.0)
        self.declare_parameter('vision_y_mm', 0.0)
        self.declare_parameter('vision_z_mm', 0.0)
        self.declare_parameter('bottom_x_mm', 0.0)
        self.declare_parameter('bottom_y_mm', 0.0)
        self.declare_parameter('bottom_z_mm', 0.0)
        self.declare_parameter('slot_x_mm', 0.0)
        self.declare_parameter('slot_y_mm', 0.0)
        self.declare_parameter('slot_z_mm', 0.0)

        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        seq_name = self.get_parameter('sequence').get_parameter_value().string_value
        self._tcp_name = self.get_parameter('tcp_name').get_parameter_value().string_value
        self._vision_x = self.get_parameter('vision_x_mm').get_parameter_value().double_value
        self._vision_y = self.get_parameter('vision_y_mm').get_parameter_value().double_value
        self._vision_z = self.get_parameter('vision_z_mm').get_parameter_value().double_value
        self._bottom_x = self.get_parameter('bottom_x_mm').get_parameter_value().double_value
        self._bottom_y = self.get_parameter('bottom_y_mm').get_parameter_value().double_value
        self._bottom_z = self.get_parameter('bottom_z_mm').get_parameter_value().double_value
        self._slot_x   = self.get_parameter('slot_x_mm').get_parameter_value().double_value
        self._slot_y   = self.get_parameter('slot_y_mm').get_parameter_value().double_value
        self._slot_z   = self.get_parameter('slot_z_mm').get_parameter_value().double_value

        self._cb_group = ReentrantCallbackGroup()

        p = f'/{ns}'
        self._movel_cli      = self.create_client(MoveLine,           f'{p}/motion/move_line',       callback_group=self._cb_group)
        self._movej_cli      = self.create_client(MoveJoint,          f'{p}/motion/move_joint',      callback_group=self._cb_group)
        self._stop_cli       = self.create_client(MoveStop,           f'{p}/motion/move_stop',       callback_group=self._cb_group)
        self._set_tcp_cli    = self.create_client(SetCurrentTcp,      f'{p}/tcp/set_current_tcp',    callback_group=self._cb_group)
        self._create_tcp_cli = self.create_client(ConfigCreateTcp,    f'{p}/tcp/config_create_tcp',  callback_group=self._cb_group)
        self._gripper_cli    = self.create_client(GripperSetPosition, '/gripper/set_position',       callback_group=self._cb_group)

        self.get_logger().info(f'[runner] 서비스 대기 중...')
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

        self._seq_name = seq_name
        self._is_moving_pub = self.create_publisher(Bool, '/motion/is_moving', 10)
        self._is_moving_pub.publish(Bool(data=True))  # 노드 기동 즉시 True — 타이머 지연 전 공백 방지
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
            self.get_logger().error('[runner] 사용 가능: home open_0 close_0 open_1 close_1 socket_fetch socket_return vision_fetch vision_return')
            self._is_moving_pub.publish(Bool(data=False))
            rclpy.shutdown()
            return

        self._set_tcp(self._tcp_name)

        self._is_moving_pub.publish(Bool(data=True))
        self.get_logger().info(f'[runner] 시퀀스 시작: {self._seq_name} ({len(seq)} steps)')
        ok = self._run_sequence(seq)
        if ok:
            self.get_logger().info(f'[runner] 시퀀스 완료: {self._seq_name}')
        else:
            self.get_logger().error(f'[runner] 시퀀스 실패: {self._seq_name} — 홈 복귀 시도')
            home_ok = self._run_sequence(home_seq())
            if not home_ok:
                self.get_logger().error('[runner] 홈 복귀 실패 — 수동 개입 필요. is_moving=False 보류')
                rclpy.shutdown()
                return
        self._is_moving_pub.publish(Bool(data=False))
        rclpy.shutdown()

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
            if not self._check_vision_coords('slot',   self._slot_x,   self._slot_y,   self._slot_z):
                return None
            return vision_return_seq(self._bottom_x, self._bottom_y, self._bottom_z,
                                     self._slot_x,   self._slot_y,   self._slot_z)

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

    def _run_sequence(self, steps: list) -> bool:
        for i, step in enumerate(steps):
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

    def _set_tcp(self, name: str) -> bool:
        # Doosan 컨트롤러는 TCP 명령에 success=False를 반환하는 경우가 있어 항상 시퀀스 진행
        try:
            # 1단계: TCP 등록 (이미 있으면 덮어쓰기) — 완료 대기
            if self._create_tcp_cli.service_is_ready():
                create_req = ConfigCreateTcp.Request()
                create_req.name = name
                create_req.pos  = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
                fut = self._create_tcp_cli.call_async(create_req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                self.get_logger().info(f'[runner] TCP 등록 완료: {name} pos=[0,0,160,0,0,0]')
            else:
                self.get_logger().warn('[runner] config_create_tcp 서비스 미준비 — 건너뜀')

            # 2단계: TCP 활성화 — 완료 대기 후 시퀀스 시작
            if self._set_tcp_cli.service_is_ready():
                set_req = SetCurrentTcp.Request()
                set_req.name = name
                fut = self._set_tcp_cli.call_async(set_req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                self.get_logger().info(f'[runner] TCP 활성화 완료: {name}')
            else:
                self.get_logger().warn('[runner] set_current_tcp 서비스 미준비 — 건너뜀')

        except Exception as e:
            self.get_logger().warn(f'[runner] TCP 설정 예외 (무시): {e}')

        return True  # 항상 시퀀스 진행

    def _movel(self, step, mode: int) -> bool:
        req = MoveLine.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = [min(step.vel, VEL_L) if step.vel is not None else VEL_L,
                          min(step.vel, VEL_R) if step.vel is not None else VEL_R]
        req.acc        = [min(step.acc, ACC_L) if step.acc is not None else ACC_L,
                          min(step.acc, ACC_R) if step.acc is not None else ACC_R]
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

    def _movej(self, step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(v) for v in step.pose]
        req.vel        = min(step.vel, VEL_J) if step.vel is not None else VEL_J
        req.acc        = min(step.acc, ACC_J) if step.acc is not None else ACC_J
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

    def _grip(self, step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0
        current = 400 if pulse > 450 else 0  # grip: 400mA, open/release: gripper_node 기본값

        req = GripperSetPosition.Request()
        req.position    = pulse
        req.current     = current
        req.timeout_sec = 0.0  # gripper_node 기본값 사용

        fut = self._gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            msg = res.message if res else 'timeout'
            self.get_logger().error(f'  gripper set_position 실패: {msg}')
            return False
        self.get_logger().info(f'  gripper ok — pos={res.final_position} cur={res.final_current}')
        time.sleep(0.1)  # gripper_node가 완료 확인 후 반환하므로 안착 여유만
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
