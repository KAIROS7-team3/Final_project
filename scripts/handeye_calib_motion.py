#!/usr/bin/env python3
"""
Hand-eye 캘리브레이션용 로봇 자세 이동 스크립트 (v2 — J4/J5/J6 다양성 추가)

조작:
  Enter     : 샘플 저장 완료 → 다음 자세
  s + Enter : 이 자세 스킵 (마커 안 보일 때)
  q + Enter : 즉시 종료
  Ctrl+C    : 즉시 종료
"""
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

# 캘리브레이션 자세 목록 [J1, J2, J3, J4, J5, J6] (단위: degree)
#
# 설계 원칙:
#   - 탑뷰 D455f 기준: J5 는 70~110° 범위 유지 (마커 가시성)
#   - J4 ±50°, J6 ±50° — 팔뚝 마커 기울기 다양화
#   - 5개 그룹으로 편의성 구분 (이동 경로 최소화)
#
# J1: 베이스 회전 / J2: 어깨 / J3: 팔꿈치
# J4: 손목1(팔뚝 축 회전) / J5: 손목2(상하 꺾임) / J6: 손목3(툴 회전)

CALIB_POSES = [
    # ── 그룹 A: 기준 자세 (J4/J5/J6 고정, J1~J3 sweep) ──────────────────
    [   0.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A1  홈
    [  30.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A2  J1 +30
    [  60.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A3  J1 +60
    [ -30.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A4  J1 -30
    [ -60.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A5  J1 -60
    [   0.0, -20.0,  90.0,   0.0,  90.0,   0.0],  # A6  J2 -20
    [   0.0, -35.0,  90.0,   0.0,  90.0,   0.0],  # A7  J2 -35
    [   0.0,   0.0,  75.0,   0.0,  90.0,   0.0],  # A8  J3 -15
    [   0.0,   0.0, 105.0,   0.0,  90.0,   0.0],  # A9  J3 +15
    [  30.0, -20.0,  80.0,   0.0,  90.0,   0.0],  # A10 J1+J2+J3 조합

    # ── 그룹 B: J4 변화 (팔뚝 축 회전 — 마커 기울기 변화) ────────────────
    [   0.0,   0.0,  90.0,  30.0,  90.0,   0.0],  # B1  J4 +30
    [   0.0,   0.0,  90.0, -30.0,  90.0,   0.0],  # B2  J4 -30
    [   0.0,   0.0,  90.0,  50.0,  90.0,   0.0],  # B3  J4 +50
    [   0.0,   0.0,  90.0, -50.0,  90.0,   0.0],  # B4  J4 -50
    [  30.0,   0.0,  90.0,  35.0,  90.0,   0.0],  # B5  J1+J4 조합
    [ -30.0,   0.0,  90.0, -35.0,  90.0,   0.0],  # B6  J1-J4 조합
    [   0.0, -20.0,  90.0,  30.0,  90.0,   0.0],  # B7  J2+J4 조합
    [   0.0, -20.0,  90.0, -30.0,  90.0,   0.0],  # B8  J2-J4 조합

    # ── 그룹 C: J5 변화 (손목 상하 꺾임 — 마커 앙각 변화) ────────────────
    [   0.0,   0.0,  90.0,   0.0,  75.0,   0.0],  # C1  J5 -15
    [   0.0,   0.0,  90.0,   0.0, 105.0,   0.0],  # C2  J5 +15
    [   0.0,   0.0,  90.0,   0.0,  70.0,   0.0],  # C3  J5 -20
    [   0.0,   0.0,  90.0,   0.0, 110.0,   0.0],  # C4  J5 +20
    [  30.0,   0.0,  90.0,   0.0,  75.0,   0.0],  # C5  J1+J5 조합
    [ -30.0,   0.0,  90.0,   0.0, 105.0,   0.0],  # C6  J1+J5 조합
    [   0.0, -20.0,  90.0,   0.0,  75.0,   0.0],  # C7  J2+J5 조합
    [   0.0, -20.0,  90.0,   0.0, 105.0,   0.0],  # C8  J2+J5 조합

    # ── 그룹 D: J6 변화 (툴 축 회전) ───────────────────────────────────
    [   0.0,   0.0,  90.0,   0.0,  90.0,  30.0],  # D1  J6 +30
    [   0.0,   0.0,  90.0,   0.0,  90.0, -30.0],  # D2  J6 -30
    [   0.0,   0.0,  90.0,   0.0,  90.0,  50.0],  # D3  J6 +50
    [   0.0,   0.0,  90.0,   0.0,  90.0, -50.0],  # D4  J6 -50
    [  30.0,   0.0,  90.0,   0.0,  90.0,  30.0],  # D5  J1+J6 조합
    [ -30.0,   0.0,  90.0,   0.0,  90.0, -30.0],  # D6  J1+J6 조합

    # ── 그룹 E: J4+J5+J6 복합 ────────────────────────────────────────
    [   0.0,   0.0,  90.0,  30.0,  75.0,  30.0],  # E1  복합 +
    [   0.0,   0.0,  90.0, -30.0, 105.0, -30.0],  # E2  복합 -
    [  30.0, -20.0,  90.0,  25.0,  80.0,  20.0],  # E3  위치+자세 복합
    [ -30.0, -20.0,  90.0, -25.0, 100.0, -20.0],  # E4  위치+자세 복합
    [   0.0, -30.0,  85.0,  20.0,  80.0,  20.0],  # E5  깊은 위치 복합
]

VEL = 20.0   # deg/sec
ACC = 10.0   # deg/sec²

HOME = CALIB_POSES[0]


class CalibMotionNode(Node):
    def __init__(self):
        super().__init__('handeye_calib_motion')
        self.cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.get_logger().info('move_joint 서비스 연결 대기 중...')
        self.cli.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('연결 완료.')

    def move_to(self, pos: list) -> bool:
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
        rclpy.spin_until_future_complete(self, future)
        return future.result().success


def main():
    rclpy.init()
    node = CalibMotionNode()

    total = len(CALIB_POSES)
    skipped = 0

    print('\n=== Hand-Eye 캘리브레이션 모션 스크립트 v2 ===')
    print(f'총 {total}개 자세 | 속도 {VEL} deg/sec')
    print('조작: Enter=샘플 저장 후 다음 | s=스킵 | q=종료 | Ctrl+C=즉시 종료')
    print('그룹 A(위치) / B(J4) / C(J5) / D(J6) / E(복합)\n')

    try:
        for i, pose in enumerate(CALIB_POSES):
            label = f'A{i+1}' if i < 10 else f'B{i-9}' if i < 18 else \
                    f'C{i-17}' if i < 26 else f'D{i-25}' if i < 32 else f'E{i-31}'
            print(f'[{i+1:02d}/{total}] {label}  자세: {pose}')

            if not node.move_to(pose):
                print('  ⚠ 이동 실패 — 자동 스킵')
                skipped += 1
                continue

            print('  ✓ 도달.  마커 확인 후 → Enter: 샘플 저장  |  s: 스킵  |  q: 종료')
            cmd = input('  > ').strip().lower()
            if cmd == 'q':
                print('사용자 종료.')
                break
            if cmd == 's':
                print('  → 스킵')
                skipped += 1

    except KeyboardInterrupt:
        print('\nCtrl+C — 종료합니다.')
    finally:
        collected = (i + 1) - skipped  # 대략적인 수집 수
        print(f'\n수집 완료 (약 {collected}개) / 스킵 {skipped}개')
        print('홈 자세로 복귀 중...')
        node.move_to(HOME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
