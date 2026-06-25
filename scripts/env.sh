#!/usr/bin/env bash
# 환경변수 설정 스크립트 — source 해서 사용
#
# 사용법:
#   source scripts/env.sh
#
# launch_robot.sh / system.launch.py 실행 전 이 파일을 반드시 source할 것.
# conda activate fp 상태를 전제함.
#
# 발견된 문제 이력:
#   - FINAL_PROJECT_ROOT 미설정 시 toolbox.yaml 등 config를 /home/kg/assistant/... 에서 탐색 (다른 노트북 잔류값)
#   - source install/setup.bash 가 시스템 PYTHONPATH를 앞에 추가해 conda 패키지(py_trees,
#     pymodbus, fastapi, ultralytics, cv2 4.13)가 시스템 패키지에 밀림
#   - empy 4.x 설치 시 colcon build 중 rosidl_adapter AttributeError 발생 → empy==3.3.4 고정

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 프로젝트 루트 ────────────────────────────────────────────────────────────
export FINAL_PROJECT_ROOT="$PROJECT_ROOT"

# ── ROS2 환경 ────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

# ── conda site-packages를 ROS2 시스템 경로보다 앞에 배치 ─────────────────────
# source setup.bash 후 실행해야 정확한 순서가 보장됨
CONDA_SP="$(python3 -c "import site; print(next(p for p in site.getsitepackages() if 'site-packages' in p))" 2>/dev/null || true)"
if [[ -n "$CONDA_SP" ]]; then
    export PYTHONPATH="$CONDA_SP:${PYTHONPATH:-}"
fi

# ── 작업 디렉터리 ─────────────────────────────────────────────────────────────
# vision 노드 등이 config/vision.yaml 을 상대 경로로 참조하므로 프로젝트 루트에서 실행
cd "$PROJECT_ROOT"

echo "[env] FINAL_PROJECT_ROOT=$FINAL_PROJECT_ROOT"
echo "[env] PYTHONPATH 앞단: $CONDA_SP"
echo "[env] CWD: $(pwd)"
