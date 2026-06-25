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

# dialout 그룹 추가 (/dev/plc 접근 권한)
TARGET_USER="${SUDO_USER:-$USER}"
if ! groups "$TARGET_USER" | grep -q dialout; then
    usermod -aG dialout "$TARGET_USER"
    echo "dialout 그룹 추가 완료: $TARGET_USER (재로그인 후 적용)"
else
    echo "dialout 그룹 이미 소속: $TARGET_USER"
fi

echo ""
echo "완료. 각 장치 재연결 후 아래로 확인:"
echo "  ls -l /dev/realsense   # RealSense D455f"
echo "  ls -l /dev/plc         # PLC XBC-DR14E (CH340, VID=1a86 PID=7523)"
echo "  ls -l /dev/gripper     # 그리퍼 — VID/PID 미기입 시 심링크 미생성"
echo ""
echo "⚠️  dialout 그룹은 재로그인 후 적용됩니다."
