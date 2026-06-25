#!/usr/bin/env bash
# 로봇 bringup 독립 실행 스크립트
# 카메라 UI와 별개로 유지됨 — 이 터미널은 로봇 연결 전용으로 열어두세요
#
# 사용법:
#   bash scripts/launch_robot.sh              # real 모드 (기본 IP)
#   bash scripts/launch_robot.sh 110.120.1.38

ROBOT_IP="${1:-110.120.1.38}"

# 환경변수 + ROS2 sourcing (conda PYTHONPATH 포함)
source "$(dirname "$0")/env.sh"

echo "[launch_robot] 로봇 연결: $ROBOT_IP"
exec ros2 launch motion bringup_e0509_with_gripper.launch.py \
    mode:=real \
    host:="$ROBOT_IP" \
    robot_ip:="$ROBOT_IP"
