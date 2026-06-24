#!/usr/bin/env bash
# Python 의존성 설치 스크립트
# 사용: bash scripts/install_python_deps.sh [--cpu]
#   --cpu  GPU 없는 노트북용 (torch CPU 빌드 사용)
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
    python3-dev

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

echo ""
echo "=== 설치 완료 ==="
echo "ROS2 Humble은 별도 apt 설치 필요 (https://docs.ros.org/en/humble/Installation.html)"
