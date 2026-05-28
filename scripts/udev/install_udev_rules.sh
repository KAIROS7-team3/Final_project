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
echo "완료. RealSense D455f 재연결 후 ls -l /dev/realsense 로 확인."
echo "gripper / plc 심링크는 VID/PID 기입 후 재실행 필요."
