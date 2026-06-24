#!/usr/bin/env bash
# udev 규칙 설치 스크립트
# 사용: sudo bash scripts/udev/install_udev_rules.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_DIR="/etc/udev/rules.d"

install_rule() {
    local src="$1"
    local dst="$UDEV_DIR/$(basename "$src")"
    cp "$src" "$dst"
    echo "installed: $dst"
}

install_rule "$SCRIPT_DIR/99-realsense-d455.rules"
install_rule "$SCRIPT_DIR/99-robot.rules"

udevadm control --reload-rules
udevadm trigger

echo ""
echo "완료. 각 장치 재연결 후 아래로 확인:"
echo "  ls -l /dev/realsense   # RealSense D455f"
echo "  ls -l /dev/plc         # PLC XBC-DR14E (CH340, VID=1a86 PID=7523)"
echo "  ls -l /dev/gripper     # 그리퍼 — VID/PID 미기입 시 심링크 미생성"
