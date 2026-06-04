#!/usr/bin/env python3
"""
캘리브레이션 자세 사전 점검 UI (handeye_calib_poses_checklist.txt 연동)

번호 입력 → 해당 자세로 이동 → 주변 충돌 확인
'h' → 홈 복귀 / 'q' → 종료

사용법:
  python3 scripts/handeye_pose_checker.py
"""
import os
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
import yaml

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'runtime.yaml')

def _load_calib_config() -> dict:
    try:
        with open(_CONFIG_PATH) as f:
            return yaml.safe_load(f).get('calibration', {})
    except Exception:
        return {}

_cfg = _load_calib_config()
VEL: float   = float(_cfg.get('motion_vel_deg_per_s', 20.0))
ACC: float   = float(_cfg.get('motion_acc_deg_per_s2', 10.0))
TIMEOUT: float = float(_cfg.get('service_timeout_sec', 5.0))

# ── 자세 목록 (체크리스트와 동일 순서) ────────────────────────────────────
POSES: list[tuple[str, list[float], str]] = [
    # (레이블, [J1,J2,J3,J4,J5,J6], 위험도)
    ("A1",  [  0.0,   0.0,  90.0,   0.0,  90.0,   0.0], ""),
    ("A2",  [ 30.0,   0.0,  90.0,   0.0,  90.0,   0.0], ""),
    ("A3",  [ 60.0,   0.0,  90.0,   0.0,  90.0,   0.0], "⚠️  J1 +60° 우측 스윕"),
    ("A4",  [-30.0,   0.0,  90.0,   0.0,  90.0,   0.0], ""),
    ("A5",  [-60.0,   0.0,  90.0,   0.0,  90.0,   0.0], "⚠️  J1 -60° 좌측 스윕"),
    ("A6",  [  0.0, -20.0,  90.0,   0.0,  90.0,   0.0], ""),
    ("A7",  [  0.0, -35.0,  90.0,   0.0,  90.0,   0.0], "⚠️  J2 -35° 팔 들어올림"),
    ("A8",  [  0.0,   0.0,  75.0,   0.0,  90.0,   0.0], ""),
    ("A9",  [  0.0,   0.0, 105.0,   0.0,  90.0,   0.0], ""),
    ("A10", [ 30.0, -20.0,  80.0,   0.0,  90.0,   0.0], ""),
    ("B1",  [  0.0,   0.0,  90.0,  30.0,  90.0,   0.0], ""),
    ("B2",  [  0.0,   0.0,  90.0, -30.0,  90.0,   0.0], ""),
    ("B3",  [  0.0,   0.0,  90.0,  50.0,  90.0,   0.0], ""),
    ("B4",  [  0.0,   0.0,  90.0, -50.0,  90.0,   0.0], ""),
    ("B5",  [ 30.0,   0.0,  90.0,  35.0,  90.0,   0.0], ""),
    ("B6",  [-30.0,   0.0,  90.0, -35.0,  90.0,   0.0], ""),
    ("B7",  [  0.0, -20.0,  90.0,  30.0,  90.0,   0.0], ""),
    ("B8",  [  0.0, -20.0,  90.0, -30.0,  90.0,   0.0], ""),
    ("C1",  [  0.0,   0.0,  90.0,   0.0,  75.0,   0.0], ""),
    ("C2",  [  0.0,   0.0,  90.0,   0.0, 105.0,   0.0], ""),
    ("C3",  [  0.0,   0.0,  90.0,   0.0,  70.0,   0.0], ""),
    ("C4",  [  0.0,   0.0,  90.0,   0.0, 110.0,   0.0], ""),
    ("C5",  [ 30.0,   0.0,  90.0,   0.0,  75.0,   0.0], ""),
    ("C6",  [-30.0,   0.0,  90.0,   0.0, 105.0,   0.0], ""),
    ("C7",  [  0.0, -20.0,  90.0,   0.0,  75.0,   0.0], ""),
    ("C8",  [  0.0, -20.0,  90.0,   0.0, 105.0,   0.0], ""),
    ("D1",  [  0.0,   0.0,  90.0,   0.0,  90.0,  30.0], ""),
    ("D2",  [  0.0,   0.0,  90.0,   0.0,  90.0, -30.0], ""),
    ("D3",  [  0.0,   0.0,  90.0,   0.0,  90.0,  50.0], ""),
    ("D4",  [  0.0,   0.0,  90.0,   0.0,  90.0, -50.0], ""),
    ("D5",  [ 30.0,   0.0,  90.0,   0.0,  90.0,  30.0], ""),
    ("D6",  [-30.0,   0.0,  90.0,   0.0,  90.0, -30.0], ""),
    ("E1",  [  0.0,   0.0,  90.0,  30.0,  75.0,  30.0], ""),
    ("E2",  [  0.0,   0.0,  90.0, -30.0, 105.0, -30.0], ""),
    ("E3",  [ 30.0, -20.0,  90.0,  25.0,  80.0,  20.0], ""),
    ("E4",  [-30.0, -20.0,  90.0, -25.0, 100.0, -20.0], ""),
    ("E5",  [  0.0, -30.0,  85.0,  20.0,  80.0,  20.0], "⚠️  J2 -30° 팔 들어올림"),
]

HOME = POSES[0][1]  # A1


class PoseCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__('handeye_pose_checker')
        self.cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.get_logger().info('move_joint 서비스 연결 대기 중...')
        if not self.cli.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError('move_joint 서비스 연결 실패')
        self.get_logger().info('연결 완료.')

    def move_to(self, pos: list[float]) -> bool:
        req = MoveJoint.Request()
        req.pos    = [float(p) for p in pos]
        req.vel    = VEL
        req.acc    = ACC
        req.time   = 0.0
        req.radius = 0.0
        req.mode   = 0
        req.blend_type = 0
        req.sync_type  = 0
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error('move_joint 응답 없음')
            return False
        return bool(result.success)


def _print_table() -> None:
    groups = [
        ("A", range(0,  10), "J1~J3 변화"),
        ("B", range(10, 18), "J4 변화"),
        ("C", range(18, 26), "J5 변화"),
        ("D", range(26, 32), "J6 변화  ← 교시패널 마커 사용 시 스킵 권장"),
        ("E", range(32, 37), "J4+J5+J6 복합"),
    ]
    print("\n" + "="*66)
    print("  번호  레이블   J1      J2      J3      J4     J5     J6")
    print("="*66)
    for grp, rng, desc in groups:
        print(f"  ── 그룹 {grp}: {desc}")
        for i in rng:
            label, joints, warn = POSES[i]
            j = joints
            warn_str = f"  {warn}" if warn else ""
            print(f"  [{i+1:02d}]   {label:<4}  "
                  f"{j[0]:6.1f}  {j[1]:6.1f}  {j[2]:6.1f}  "
                  f"{j[3]:5.1f}  {j[4]:5.1f}  {j[5]:5.1f}{warn_str}")
    print("="*66)
    print("  조작: 번호 입력 → 이동 | h → 홈(A1) | l → 목록 | q → 종료")
    print("="*66 + "\n")


def main() -> None:
    rclpy.init()
    node = PoseCheckerNode()

    _print_table()

    try:
        while True:
            try:
                raw = input("번호 입력 > ").strip().lower()
            except EOFError:
                break

            if raw == 'q':
                break

            if raw == 'l':
                _print_table()
                continue

            if raw == 'h':
                print("  → 홈(A1)으로 복귀 중...")
                ok = node.move_to(HOME)
                print(f"  {'✓ 완료' if ok else '✗ 실패'}")
                continue

            if not raw.isdigit():
                print("  숫자, h, l, q 중 하나를 입력하세요.")
                continue

            num = int(raw)
            if not (1 <= num <= len(POSES)):
                print(f"  1~{len(POSES)} 범위로 입력하세요.")
                continue

            label, joints, warn = POSES[num - 1]
            print(f"  → [{num:02d}] {label}  {joints}")
            if warn:
                print(f"  {warn}  —  주변 공간 확인 후 Enter, 취소는 Ctrl+C")
                try:
                    input("     ")
                except KeyboardInterrupt:
                    print("\n  취소됨.")
                    continue

            ok = node.move_to(joints)
            print(f"  {'✓ 도달' if ok else '✗ 이동 실패'}\n")

    except KeyboardInterrupt:
        print('\n종료합니다.')
    finally:
        print('홈(A1)으로 복귀 중...')
        node.move_to(HOME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
