#!/usr/bin/env bash
# Python 의존성 설치 스크립트
#
# 사용법:
#   bash scripts/install_python_deps.sh        # GPU (CUDA 12.1) 빌드
#   bash scripts/install_python_deps.sh --cpu  # CPU 빌드 (CUDA 없는 노트북)
#
# 전제: conda activate fp 상태에서 실행
set -euo pipefail

CPU_ONLY=false
for arg in "$@"; do
    [[ "$arg" == "--cpu" ]] && CPU_ONLY=true
done

echo "=== apt 시스템 의존성 ==="
sudo apt-get update -q
sudo apt-get install -y \
    portaudio19-dev \
    python3-pip \
    python3-dev \
    ros-humble-vision-msgs \
    ros-humble-v4l2-camera

# brltty가 CH340 USB-Serial을 낚아채 /dev/ttyUSB* 미생성 문제 방지
if dpkg -l brltty &>/dev/null; then
    sudo apt-get remove -y brltty
    echo "brltty 제거 완료 (CH340 PLC 인식 충돌 방지)"
fi

echo ""
echo "=== colcon 빌드용 empy 고정 (ROS2 Humble은 3.x API 필요) ==="
pip install "empy==3.3.4"

echo ""
echo "=== pip 패키지 설치 ==="

# torch는 CPU/GPU 빌드가 다르므로 먼저 설치
if $CPU_ONLY; then
    echo "[torch] CPU 빌드 설치..."
    pip install torch==2.12.0 torchvision==0.27.0 \
        --index-url https://download.pytorch.org/whl/cpu
else
    echo "[torch] GPU(CUDA 12.1) 빌드 설치..."
    pip install torch==2.12.0 torchvision==0.27.0 \
        --index-url https://download.pytorch.org/whl/cu121
fi

# 나머지 의존성 (torch 제외)
pip install -r "$(dirname "$0")/../requirements.txt" \
    --ignore-installed torch torchvision

# pandas 의존성 (conda fp 환경에 pandas가 있을 경우 필요)
pip install pytz

echo ""
echo "=== RealSense USB autosuspend 비활성화 ==="
echo "  (재부팅 시 초기화됨 — 영구 적용은 /etc/rc.local 또는 udev 규칙 사용)"
sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend" || true

echo ""
echo "=== 설치 완료 ==="
echo "ROS2 Humble은 별도 apt 설치 필요 (https://docs.ros.org/en/humble/Installation.html)"
echo "환경 확인: source scripts/env.sh && ros2 doctor"
