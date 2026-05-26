#!/usr/bin/env bash
#
# Track Selector — 3-트랙 중 하나를 선택해 실행한다.
#
# 사용법:
#   ./run.sh --track A     # Gemma 4 + BT + DSR 좌표 제어 (ROS2)
#   ./run.sh --track B     # Gemma 4 + BT + RL 정책 (ROS2)
#   ./run.sh --track C     # 키워드 파서 + VLA + Doosan Python SDK (no ROS2)
#
# 옵션:
#   --sim          시뮬레이션 모드 (Gazebo, hardware 없음)
#   --no-watchdog  SafetyWatchdog 비활성화 (테스트 전용 — production 금지, S-4)
#   --dry-run      실행 명령만 출력하고 종료
#
# 사전조건 (CLAUDE.md, .claude/rules/safety.md S-1~S-9):
#   - Track C 시작 전 ROS2 stack 완전 종료 + VRAM 해제 확인
#   - .env 파일 존재 (cp .env.example .env)
#   - config/*.yaml 5개 파일 존재 + 캘리브레이션 완료
#   - 부팅 시 YOLOv8 reconciliation 통과 (S-9)
#
# 참조: CLAUDE.md, docs/architecture.md, .claude/rules/safety.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ─── 1. 인자 파싱 (먼저 처리 — --help는 .env 없이도 동작) ──────
TRACK=""
SIM=0
NO_WATCHDOG=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --track)        TRACK="$2"; shift 2 ;;
        --sim)          SIM=1; shift ;;
        --no-watchdog)  NO_WATCHDOG=1; shift ;;
        --dry-run)      DRY_RUN=1; shift ;;
        -h|--help)      sed -n '2,20p' "$0"; exit 0 ;;
        *)              echo "❌ 알 수 없는 인자: $1" >&2; exit 1 ;;
    esac
done

if [[ -z "$TRACK" ]]; then
    echo "❌ --track [A|B|C] 필수 (--help 참조)" >&2
    exit 1
fi

if [[ "$TRACK" != "A" && "$TRACK" != "B" && "$TRACK" != "C" ]]; then
    echo "❌ --track 값은 A, B, C 중 하나" >&2
    exit 1
fi

if [[ $NO_WATCHDOG -eq 1 ]]; then
    echo "⚠️  WARNING: SafetyWatchdog 비활성화 — production 금지 (S-4)" >&2
    read -p "정말 계속하시겠습니까? (yes/no): " confirm
    [[ "$confirm" == "yes" ]] || exit 1
fi

# ─── 2. 환경 변수 로드 ─────────────────────────────────────────
if [[ ! -f .env ]]; then
    echo "❌ .env 파일이 없습니다. 'cp .env.example .env' 후 값을 채워주세요." >&2
    exit 1
fi
set -a; source .env; set +a

# ─── 3. Pre-flight 체크 ────────────────────────────────────────
echo "🔍 Pre-flight 체크..."

# config 파일 존재
for f in staging_area.yaml toolbox.yaml hand_eye.yaml robot_poses.yaml fod.yaml runtime.yaml; do
    [[ -f "config/$f" ]] || { echo "❌ config/$f 누락" >&2; exit 1; }
done

# 트랙별 사전 종료 + VRAM 확인
if [[ "$TRACK" == "C" ]]; then
    echo "  → ROS2 stack 종료 확인 (Track C는 ROS2 사용 안 함)"
    # TODO(Phase 0 ①): ros2 daemon stop + 모든 ros2 노드 PID kill
    if pgrep -f "ros2|_ros2_daemon" > /dev/null; then
        echo "❌ ROS2 프로세스 감지. 종료 후 재시도:" >&2
        echo "    ros2 daemon stop; pkill -f ros2" >&2
        exit 1
    fi

    echo "  → VRAM 사용량 확인"
    if command -v nvidia-smi > /dev/null; then
        VRAM_USED_MB=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
        # Track C 모델 로딩 전이므로 사용량이 충분히 낮아야 함 (임계값: 2GB)
        if (( VRAM_USED_MB > 2048 )); then
            echo "⚠️  VRAM 사용량 ${VRAM_USED_MB}MB — 다른 GPU 프로세스 종료 권장" >&2
        fi
    fi
else
    # Track A/B는 ROS2 필요
    echo "  → ROS2 환경 확인"
    if ! command -v ros2 > /dev/null; then
        echo "❌ ROS2 미설치 또는 환경 미로드. 'source /opt/ros/humble/setup.bash'" >&2
        exit 1
    fi
fi

# 하드웨어 연결 확인 (실모드)
if [[ $SIM -eq 0 ]]; then
    echo "  → 하드웨어 연결 확인"
    # TODO(Phase 0 ②): Doosan 로봇 ping, RealSense enumerate, PLC ping
    : # placeholder — 실제 체크는 hardware.md 참조
fi

echo "✅ Pre-flight OK"

# ─── 4. Track 실행 ─────────────────────────────────────────────
case "$TRACK" in
    A|B)
        LAUNCH_FILE="orchestrator_track_${TRACK,,}.launch.py"
        if [[ $SIM -eq 1 ]]; then LAUNCH_FILE="sim_${LAUNCH_FILE}"; fi
        CMD="ros2 launch orchestrator $LAUNCH_FILE"
        # TODO(Phase 0 ①): launch 파일 작성 후 위 경로 확정
        ;;
    C)
        CMD="python3 track_c_vla.py"
        [[ $SIM -eq 1 ]] && CMD="$CMD --sim"
        # TODO(Phase 0 ①): track_c_vla.py 골격 작성
        ;;
esac

echo "▶ Track $TRACK 실행: $CMD"

if [[ $DRY_RUN -eq 1 ]]; then
    echo "(--dry-run — 실행하지 않음)"
    exit 0
fi

# ─── 5. 실행 ───────────────────────────────────────────────────
# trap으로 종료 시 정리 (PLC 빨강 + DB 로그)
cleanup() {
    echo ""
    echo "🛑 종료 처리 중..."
    # TODO: PLC를 red solid로 (S-3), DB에 system_events 'shutdown' 기록
}
trap cleanup EXIT

exec $CMD
