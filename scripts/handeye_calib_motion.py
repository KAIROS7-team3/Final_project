#!/usr/bin/env python3
"""
Hand-eye 캘리브레이션용 로봇 자세 이동 스크립트 (v2 — J4/J5/J6 다양성 추가, 37자세)

⚠️ E-1 예외: Doosan MoveJoint API(dsr_msgs2.srv.MoveJoint)는 degree 입력을 요구하는
하드웨어 경계 인터페이스다. CALIB_POSES와 vel/acc 단위가 degree/deg·s⁻¹인 이유.
vel/acc 기본값은 config/runtime.yaml calibration 섹션에서 로드한다.

조작:
  Enter     : 샘플 저장 완료 → 다음 자세
  s + Enter : 이 자세 스킵 (마커 안 보일 때)
  q + Enter : 즉시 종료
  Ctrl+C    : 즉시 종료
"""
import logging
import os
import sys

import rclpy
import yaml
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('handeye_calib_motion')

# ── config 로드 ────────────────────────────────────────────────────────────
_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'runtime.yaml')

def _load_calib_config() -> dict:
    try:
        with open(_CONFIG_PATH) as f:
            return yaml.safe_load(f).get('calibration', {})
    except Exception as e:
        logger.warning('config/runtime.yaml 로드 실패, 기본값 사용: %s', e)
        return {}

_cfg = _load_calib_config()
VEL: float = float(_cfg.get('motion_vel_deg_per_s', 20.0))
ACC: float = float(_cfg.get('motion_acc_deg_per_s2', 10.0))
SERVICE_TIMEOUT: float = float(_cfg.get('service_timeout_sec', 5.0))

from calib_poses import CALIB_POSES, GROUPS, HOME  # 공유 자세 목록


class CalibMotionNode(Node):
    def __init__(self) -> None:
        super().__init__('handeye_calib_motion')
        self.cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.get_logger().info('move_joint 서비스 연결 대기 중...')
        if not self.cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error(f'move_joint 서비스 연결 실패 (timeout={SERVICE_TIMEOUT:.1f}s)')
            raise RuntimeError('move_joint service unavailable')
        self.get_logger().info('연결 완료.')

    def move_to(self, pos: list[float]) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos]
        req.vel = VEL
        req.acc = ACC
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0        # ABSOLUTE
        req.blend_type = 0
        req.sync_type = 0   # SYNC
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('move_joint 타임아웃 또는 응답 없음')
            return False
        return bool(future.result().success)


def main() -> None:
    rclpy.init()
    node = CalibMotionNode()

    total = len(CALIB_POSES)
    skipped = 0
    i = -1  # finally 블록에서 루프 미실행 시 UnboundLocalError 방지

    print(f'\n=== Hand-Eye 캘리브레이션 모션 스크립트 v2 ===')
    print(f'총 {total}개 자세 | 속도 {VEL} deg/s | 가속도 {ACC} deg/s²')
    print('조작: Enter=샘플 저장 후 다음 | s=스킵 | q=종료 | Ctrl+C=즉시 종료')
    print('그룹 A(위치) / B(J4) / C(J5) / D(J6) / E(복합)\n')

    _boundaries: list[int] = []
    _acc = 0
    for _, count in GROUPS:
        _acc += count
        _boundaries.append(_acc)

    def _label(idx: int) -> str:
        offset = 0
        for (name, _), boundary in zip(GROUPS, _boundaries):
            if idx < boundary:
                return f'{name}{idx - offset + 1}'
            offset = boundary
        return f'?{idx + 1}'

    try:
        for i, pose in enumerate(CALIB_POSES):
            label = _label(i)

            print(f'[{i + 1:02d}/{total}] {label}  자세: {pose}')

            if not node.move_to(pose):
                print('  ⚠ 이동 실패 — 자동 스킵')
                skipped += 1
                continue

            print('  ✓ 도달.  마커 확인 후 → Enter: 샘플 저장  |  s: 스킵  |  q: 종료')
            cmd = input('  > ').strip().lower()
            if cmd == 'q':
                print('사용자 종료.')
                skipped += 1  # 이동은 완료됐으나 샘플 미저장
                break
            if cmd == 's':
                print('  → 스킵')
                skipped += 1

    except KeyboardInterrupt:
        print('\nCtrl+C — 종료합니다.')
    finally:
        collected = (i + 1) - skipped if i >= 0 else 0
        print(f'\n수집 완료 (약 {collected}개) / 스킵 {skipped}개')
        print('홈 자세로 복귀 중...')
        node.move_to(HOME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
