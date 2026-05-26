# 엔지니어링 룰

> 🟠 코딩 컨벤션, 단위/좌표, 의존성, 에러 처리, 로깅 표준.

## E-1. 단위 및 좌표

| 항목 | 표준 |
|------|------|
| 각도 / joint command | **rad** (degree 금지) |
| 길이 | **m** (mm 금지) |
| 시간 | **ROS2 Time (rclpy.time.Time)** — 절대 시각, monotonic 아님 |
| 좌표계 | **robot_base_link** frame 기본. 다른 frame 사용 시 변수명에 frame 명시 (`pose_camera`, `pose_world`) |
| 회전 표현 | **쿼터니언 (x, y, z, w)**. 오일러 각 사용 시 변환 함수 + 단위 명시 |

## E-2. 의존성 그래프 (Import 룰)

| 모듈 | 허용 import | 금지 import |
|------|-------------|-------------|
| `db_core/` | 표준 라이브러리, DB 드라이버 | `rclpy`, `interfaces`, ROS2 |
| `plc_core/` | 표준 라이브러리, Modbus 드라이버 | `rclpy`, `interfaces`, ROS2 |
| `unit_actions/` | 표준 라이브러리, `hal` | `rclpy`, `interfaces`, ROS2 |
| `track_c_vla.py` | `db_core`, `plc_core`, `pyrealsense2`, Doosan Python SDK, VLA 라이브러리 | `rclpy`, `interfaces`, `unit_actions`, `hal` |
| ROS2 패키지 (`voice/`, `vision/` 등) | `rclpy`, `interfaces`, 동일 워크스페이스 패키지 | 다른 패키지 내부 모듈 직접 import |

CI에서 `grep -r 'import rclpy' db_core/ plc_core/ unit_actions/`로 검사 자동화 가능.

## E-3. 패키지 간 통신

- ROS2 패키지 간 통신은 **`interfaces/`의 msg/srv/action 통해서만**
- 다른 패키지의 내부 모듈을 직접 import 금지
- 새 통신 인터페이스 추가 → `interface-guardian` 에이전트 검토 + `interfaces/CHANGELOG.md` 갱신

## E-4. 설정 관리

- **좌표, 임계값, 시간 상수는 모두 `config/*.yaml`**. 코드 내 하드코딩 금지
- 환경별 다른 값(개발/스테이징/프로덕션)은 `.env` 또는 환경 변수
- 시크릿(API 키, 클라우드 토큰)은 `.env`에 저장 — git commit 금지

| 설정 파일 | 용도 |
|-----------|------|
| `config/staging_area.yaml` | 공구별 Staging Area 좌표 |
| `config/toolbox.yaml` | 슬롯 좌표 + 공구 기하 |
| `config/hand_eye.yaml` | 카메라-엔드이펙터 변환 행렬 |
| `config/robot_poses.yaml` | home, scan 포즈 |
| `config/fod.yaml` | FOD 임계 시간 등 운영 파라미터 |
| `config/runtime.yaml` | 비-시크릿 상수 (robot_model, whisper_model_size, operator_id) |
| `.env` (gitignored) | API 키, 비밀 토큰 |

## E-5. 에러 처리

- 모든 actuator 호출(`arm.execute`, `gripper.set`, `plc.set_state`)은 try/except 필수
- 실패 시 반드시 다음 3가지 수행:
  1. **DB 로그** — `event_type='error'` 또는 'rejected', `notes`에 사유
  2. **PLC 상태 갱신** — 빨간 점멸 또는 적절한 경고
  3. **운영자 안내** — 로그 또는 UI

```python
# ✅ 올바른 패턴
try:
    arm.execute(joint_traj)
    db.log_event(tool_id=tool_id, event_type='fetch', track='C')
except DoosanSDKError as e:
    db.log_event(tool_id=tool_id, event_type='error', track='C', notes=str(e))
    plc.set_error()
    raise
```

- silent fallback 금지 (예외를 잡아서 무시하면 안 됨)
- retry는 명시적으로 횟수 + backoff 지정. 무한 retry 금지

## E-6. 로깅 표준

### DB 이벤트 로깅
- 모든 fetch/return/error/rejected 이벤트는 `tool_events` 테이블에 기록
- `event_type`은 스키마 정의된 enum만 사용 (임의 문자열 금지)
- `track` 필드 항상 채움 ('A' / 'B' / 'C')
- `operator_id`는 v1.0에서 `'operator_01'` 고정

### Python 로깅
- `logging` 모듈 사용. `print()` 금지 (테스트 fixture 제외)
- 레벨: DEBUG (개발), INFO (정상 흐름), WARNING (회복 가능), ERROR (회복 불가)
- 메시지 형식: `[모듈명] 이벤트 - context (key=value)`

```python
logger.info("[grasp] success - tool_id=%s slot=(%d,%d)", tool_id, row, col)
logger.error("[motion] joint limit violation - joint=%d value=%.3f", j, v)
```

### ROS2 로깅
- `node.get_logger().info(...)` 사용
- 동일한 레벨 매핑 (debug/info/warn/error)

## E-7. 코딩 스타일 (Python)

- **PEP 8** 준수. 자동 포매터 `ruff format` (또는 `black`) 적용
- **타입 힌트 필수**: 모든 public 함수의 인자 + 반환값
- **f-string 사용**. `.format()` / `%` 사용 자제
- **dataclass** for value objects, **TypedDict** for dict shapes
- 함수 길이 50줄 이하 권장. 초과 시 분리 검토

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class GraspPose:
    position: tuple[float, float, float]  # (x, y, z) in m, robot_base_link frame
    quaternion: tuple[float, float, float, float]  # (x, y, z, w)
    approach_dir: tuple[float, float, float]
```

## E-8. 명명 규칙

| 대상 | 규칙 | 예시 |
|------|------|------|
| Python 모듈/함수/변수 | `snake_case` | `fetch_tool`, `tool_pose` |
| Python 클래스 | `PascalCase` | `BehaviorTree`, `SafetyValidator` |
| 상수 | `UPPER_SNAKE_CASE` | `MAX_JOINT_VEL`, `FOD_TIMEOUT_MIN` |
| ROS2 노드 | `snake_case` | `whisper_node`, `gemma_intent_node` |
| ROS2 메시지 | `PascalCase` | `ToolStatus`, `HandoverEvent` |
| ROS2 토픽 | `/group/topic` | `/voice/raw_text`, `/vision/detections` |
| ROS2 서비스 | `/group/CamelCase` | `/db/CheckToolFeasibility` |
| 공구 ID | `<type>_<spec>` | `screwdriver_phillips_small`, `wrench_8mm` |
| config 키 | `snake_case` | `checkout_timeout_minutes` |

## E-9. 비동기 / 동시성

- ROS2 노드 내 콜백은 빠르게 반환 (블로킹 작업은 별도 스레드/액션 사용)
- Track C `track_c_vla.py`는 `asyncio` 패턴 사용 (STT는 async, VLA 추론은 동기)
- `is_moving` 플래그는 단일 작성자 (모션 시작/종료 시점)에서만 변경
- 공유 상태 접근은 명시적 락 또는 단일 스레드 보장
