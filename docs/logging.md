# 로깅 인프라

> 시스템 전체의 로깅 계층, 파일 위치, 순환 정책, 민감 정보 처리를 정의한다.
> 코드 작성 시 [`.claude/rules/engineering.md` E-6](../.claude/rules/engineering.md) 로깅 표준과 함께 참조.

---

## 1. 로깅 계층 개요

이 시스템은 세 가지 독립적인 로깅 계층을 사용한다:

| 계층 | 구현 | 대상 트랙 | 저장 위치 |
|------|------|-----------|----------|
| **DB 이벤트 로그** | `db_core/` → SQLite WAL | A, B, C | DB 파일 (구조화) |
| **Python 파일 로그** | `logging` 모듈 (RotatingFileHandler) | A, B, C | `~/.robot/logs/` |
| **ROS2 로그** | `rclpy` logger → `~/.ros/log/` | A, B만 | `~/.ros/log/` (자동) |

세 계층은 상호 보완적이다:
- **DB 이벤트**: 공구 상태 변화·명령·오류의 구조화된 감사 기록 (분석·디버깅 기준)
- **Python 파일 로그**: 런타임 흐름·예외·모듈 상태 (개발·운영 디버깅)
- **ROS2 로그**: 노드 간 통신·타이밍·BT 상태 (Track A/B 전용)

---

## 2. DB 이벤트 로그

DB 이벤트 로그는 공구 관련 모든 행위의 **단일 진실**이다. 상세 스키마 → [`docs/db-schema.md`](db-schema.md).

### 기록 대상

| 테이블 | 기록 시점 |
|--------|----------|
| `tool_events` | fetch / return / rejected / error / fod_alert / reconciled 발생마다 |
| `system_events` | 부팅 / E-stop / DB 캐시 폴백 / 캘리브레이션 시작·완료 |

### 필수 필드

```python
db.log_event(
    tool_id    = "screwdriver_phillips_small",
    event_type = "fetch",   # enum만 허용 (E-6)
    track      = "A",       # "A" / "B" / "C"
    notes      = None,      # 오류 시 상세 메시지
)
```

> `operator_id`는 v1.0에서 `'operator_01'` 고정 (결정 #20). `db_core/`가 자동 채움.

---

## 3. Python 파일 로그

### 3.1 로그 디렉토리

```
~/.robot/logs/
├── track_a.log        ← Track A 런타임 (rclpy 제외)
├── track_b.log        ← Track B 런타임
├── track_c.log        ← Track C (track_c_vla.py)
├── db_core.log        ← db_core/ 모듈
├── plc_core.log       ← plc_core/ 모듈
└── safety.log         ← SafetyValidator / SafetyWatchdog (별도 파일 — 안전 감사용)
```

> `~/.robot/logs/`는 첫 실행 시 자동 생성. 위치는 `config/runtime.yaml`의 `log_dir`로 재정의 가능.

### 3.2 로거 초기화 패턴

```python
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

def get_logger(name: str) -> logging.Logger:
    log_dir = Path.home() / ".robot" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(
            log_dir / f"{name}.log",
            maxBytes=10 * 1024 * 1024,  # 10 MB
            backupCount=5,
        )
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s [%(name)s] %(message)s",
            datefmt="%Y-%m-%dT%H:%M:%S",
        ))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger
```

### 3.3 로그 레벨 기준

| 레벨 | 사용 기준 | 예시 |
|------|----------|------|
| `DEBUG` | 개발·디버깅용 상세 흐름 | joint 값, 프레임별 추론 결과 |
| `INFO` | 정상 동작 흐름 이정표 | "fetch 완료", "홈 복귀" |
| `WARNING` | 자동 회복 가능한 이상 | "DB 캐시 폴백 활성화", "VRAM 사용량 경고" |
| `ERROR` | 회복 불가, 운영자 개입 필요 | "그리퍼 응답 없음", "joint limit 초과" |

> `print()` 사용 금지 (테스트 fixture 제외). E-6 참조.

### 3.4 메시지 형식

```python
# ✅ 올바름 — 모듈명 + 이벤트 + context
logger.info("[grasp] success - tool_id=%s slot=(%d,%d)", tool_id, row, col)
logger.error("[motion] joint limit violation - joint=%d value=%.4f rad", j, v)
logger.warning("[db] cache fallback active - age_s=%.1f", age)

# ❌ 금지 — 정보 없는 메시지
logger.info("done")
logger.error("error occurred")
```

---

## 4. ROS2 로그 (Track A/B 전용)

Track A/B 노드는 `rclpy` 내장 로거를 사용한다.

```python
# ROS2 노드 내
self.get_logger().info("[whisper_node] STT result: %s", text)
self.get_logger().error("[orchestrator] BT tick failed: %s", str(e))
```

ROS2 로그는 `~/.ros/log/<session>/` 에 자동 저장. `colcon test` 실행 시도 동일 경로.

### ROS2 로그 레벨 매핑

| rclpy | Python logging | 의미 |
|-------|---------------|------|
| `debug` | `DEBUG` | 동일 |
| `info` | `INFO` | 동일 |
| `warn` | `WARNING` | 동일 |
| `error` | `ERROR` | 동일 |
| `fatal` | — | 프로세스 종료 직전만 사용 |

---

## 5. 로그 순환 및 보존

### Python 파일 로그

| 파일 | 최대 크기 | 보존 세대 | 압축 |
|------|----------|----------|------|
| `safety.log` | 10 MB | 10세대 | 권장 (gzip) |
| `track_*.log` | 10 MB | 5세대 | 선택 |
| `db_core.log`, `plc_core.log` | 10 MB | 5세대 | 선택 |

> `RotatingFileHandler`가 자동 순환. 추가 아카이빙은 미결 #34 결정 후.

### ROS2 로그

ROS2 기본 설정으로 세션별 디렉토리 생성 후 자동 보존. `~/.ros/log/` 용량 주기적 확인 권장:

```bash
du -sh ~/.ros/log/
# 오래된 세션 삭제
ros2 run ros2doctor delete_log
```

### DB 이벤트 로그

DB 파일 자체가 영구 기록. 보존 기간·archival 정책은 미결 #34.

---

## 6. 모니터링 대시보드 (HP ProBook)

HP ProBook 450 G10은 모니터링 전용으로 제어 권한 없음 (결정 #28).

Phase 8+에서 구현 예정인 대시보드 후보:

| 항목 | 표시 내용 |
|------|----------|
| 공구 상태 | `tools.current_status` 실시간 |
| 최근 이벤트 | `tool_events` 최근 20건 |
| 시스템 이벤트 | E-stop / reconciliation mismatch |
| 오류율 | 트랙별 `error` / `rejected` 비율 |
| VRAM | `nvidia-smi` 실시간 |

> 대시보드 구현 방식 (Grafana / 자체 웹 / ROS2 rqt) 미결정.

---

## 7. 민감 정보 처리

로그에 절대 기록하지 않을 항목:

- API 키, 토큰 (`.env` 변수값)
- 운영자 개인 식별 정보 (v1.0: 해당 없음)
- 카메라 스트림 원본 이미지 (로그 파일 내)

```python
# ❌ 금지
logger.debug("HuggingFace token: %s", os.environ["HUGGINGFACE_TOKEN"])

# ✅ 올바름
logger.debug("HuggingFace token loaded: length=%d", len(token))
```

---

## 8. 미결 / 후속 작업

| 항목 | 결정 시점 |
|------|----------|
| 로그 보존 기간 및 archival 정책 | Phase 7 전 (미결 #34) |
| 외부 집계 도구 (ELK Stack / Grafana Loki / 없음) | Phase 7 전 (미결 #34) |
| 모니터링 대시보드 구현 방식 | Phase 8+ |
| 로그 경보 (error 빈도 임계 초과 시 알림) | Phase 8+ |
