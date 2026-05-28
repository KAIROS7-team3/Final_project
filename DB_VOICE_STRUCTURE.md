# DB / Voice 구현 구조안

이 문서는 담당 범위인 `ros2_ws/src/db`와 `ros2_ws/src/voice`를 어떻게 구성할지 정리한다.
`ros2_ws/src/plc`는 이미 완성된 영역으로 보고, 명시 요청 없이는 수정하지 않는다.

참조 기준:
- `AGENTS.md`
- `docs/architecture.md`
- `docs/interfaces.md`
- `docs/db-schema.md`
- `docs/logging.md`
- `.claude/skills/whisper-stt/SKILL.md`

---

## 1. 전체 방향

Track A/B 담당 범위는 `ros2_ws/src/db`와 `ros2_ws/src/voice` 안에서 닫는다.
루트의 `*_core/` 계열은 다른 담당 영역으로 보고 참조, 수정, 재사용 대상으로 삼지 않는다.

현재 상태:
- `ros2_ws/src/db`: 비어 있음 (`.gitkeep`)
- `ros2_ws/src/voice`: 비어 있음 (`.gitkeep`)
- `ros2_ws/src/plc`: 비어 있지만 PLC는 별도 완성본이 있으므로 변경 대상 아님

구현 원칙:
- `*_core/`는 보지 않고 건드리지 않는다.
- `ros2_ws/src/db`는 ROS2 서비스 서버와 DB 접근 로직을 자체 포함한다.
- `ros2_ws/src/voice`는 Whisper STT, Gemma intent, moving gate를 담당한다.
- 모든 `fetch`/`return`은 DB Gate를 통과해야 한다.
- `is_moving=True` 동안 음성 추론과 명령 발행을 차단한다.
- boot reconciliation 완료 전에는 모든 명령을 거부한다.

---

## 2. 목표 패키지 구조

```text
ros2_ws/src/db/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── db
├── db/
│   ├── __init__.py
│   ├── db_service_node.py
│   ├── fod_monitor_node.py
│   └── reconciliation_gate.py
├── launch/
│   └── db.launch.py
└── test/
    ├── test_db_service_node.py
    └── test_fod_monitor_node.py

ros2_ws/src/voice/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── voice
├── voice/
│   ├── __init__.py
│   ├── whisper_node.py
│   ├── gemma_intent_node.py
│   ├── wake_word_detector.py
│   ├── audio_gate.py
│   └── intent_schema.py
├── launch/
│   └── voice.launch.py
└── test/
    ├── test_audio_gate.py
    ├── test_intent_schema.py
    └── test_gemma_intent_node.py
```

---

## 3. DB 패키지 책임

### `db_service_node.py`

ROS2 서비스 서버를 제공한다.

제공 서비스:
- `CheckToolFeasibility.srv`
- `UpdateToolStatus.srv`

역할:
- SQLite DB에서 `tools.current_status`를 조회해 feasibility 판단
- 허용 불가 명령은 `rejected` 이벤트로 DB에 기록
- 상태 변경 요청은 `tool_events`에 append-only로 기록하고 `tools.current_status` 갱신
- DB 장애 시 캐시 TTL 정책을 적용하고, TTL 초과면 모든 명령 거부
- boot reconciliation이 끝나기 전에는 항상 `feasible=False` 반환

주의:
- 서비스 콜백에서 DB 예외를 삼키지 않는다.
- 실패 응답에는 운영자가 이해할 수 있는 `reason` 또는 `message`를 채운다.
- `track` 값은 `A`, `B`, `C` 중 하나만 허용한다.

### `fod_monitor_node.py`

FOD 상태 전이를 감시한다.

역할:
- 주기적으로 `tools.current_status`와 마지막 이벤트 시각 조회
- `out` 또는 `staged` 상태가 `config/fod.yaml` 임계 시간을 넘으면 `missing` 또는 `fod_alert`로 전이
- 전이 시 `tool_events`에 `fod_alert` 이벤트 기록
- PLC 알림은 PLC 완성본 또는 orchestrator 연계 지점으로 넘긴다

기본 규칙:
- FOD 임계 시간 기본값은 10분
- FOD 알림 지연 목표는 30초 이내
- 상태 전이는 DB 이벤트 로그와 `tools.current_status`가 항상 일치해야 한다.

### `reconciliation_gate.py`

부팅 reconciliation 완료 여부를 DB 서비스에서 확인하기 위한 작은 게이트 모듈이다.

역할:
- `system_events.boot_complete` 또는 별도 런타임 플래그 확인
- 완료 전 `CheckToolFeasibility` 거부
- mismatch 발생 시 `system_events.reconciliation_mismatch` 기록

---

## 4. Voice 패키지 책임

### `whisper_node.py`

마이크 오디오를 텍스트로 변환해서 `/voice/raw_text`로 발행한다.

구독:
- `/robot/status` (`interfaces/RobotStatus`)

발행:
- `/voice/raw_text` (`std_msgs/String`)

역할:
- Whisper small 모델 사용
- 한국어 고정, `temperature=0.0`, `condition_on_previous_text=False`
- VAD를 먼저 통과한 음성 구간만 Whisper에 전달
- `RobotStatus.is_moving=True`이면 STT를 실행하지 않고 오디오를 무시

안전 조건:
- 로봇 이동 중에는 음성 명령을 받을 수 없어야 한다.
- moving 상태가 풀린 뒤에만 다시 수음한다.

### `gemma_intent_node.py`

STT 텍스트를 구조화된 intent로 변환해서 `/voice/intent`로 발행한다.

구독:
- `/voice/raw_text` (`std_msgs/String`)
- `/vision/scene_context` (`std_msgs/String`, JSON)

클라이언트:
- `CheckToolFeasibility.srv`

발행:
- `/voice/intent` (`interfaces/Intent`)

역할:
- Gemma 4 로컬 추론으로 `fetch`, `return`, `cancel`, `unknown` 분류
- 공구명은 프로젝트 표준 `tool_id`로 정규화
- DB Gate 결과가 `feasible=True`일 때만 실행 가능한 intent 발행
- DB Gate 차단 시 operator 안내 로그를 남기고 실행 intent를 발행하지 않거나 `unknown`/거부 상태로 처리
- `/vision/scene_context`는 모호한 공구명 보정에 사용

성능 목표:
- Gemma 4 의도 정확도 97% 이상
- Gemma 4 + DB 확인 800ms 이내
- 불가 명령 100% 차단

### `wake_word_detector.py`

웨이크워드 기반으로 오인식을 줄이는 선택 모듈이다.

역할:
- 지정된 호출어 이후의 발화만 STT/intent 파이프라인에 전달
- false positive를 줄이는 전처리 계층

### `audio_gate.py`

음성 게이팅 로직을 테스트 가능하게 분리한다.

역할:
- `is_moving`
- VAD 결과
- wake word 결과
- 최소 confidence 조건

이 모듈은 ROS2 의존성을 최소화하거나 없애서 단위 테스트를 쉽게 만든다.

---

## 5. 주요 ROS2 인터페이스

### 토픽

| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/voice/raw_text` | `std_msgs/String` | `whisper_node` | `gemma_intent_node` |
| `/voice/intent` | `interfaces/Intent` | `gemma_intent_node` | `orchestrator` |
| `/robot/status` | `interfaces/RobotStatus` | `motion` | `whisper_node` |
| `/vision/scene_context` | `std_msgs/String` JSON | `vision/context_builder` | `gemma_intent_node` |

### 서비스

| 서비스 | 서버 | 클라이언트 |
|--------|------|------------|
| `CheckToolFeasibility` | `db_service_node` | `voice`, `orchestrator` |
| `UpdateToolStatus` | `db_service_node` | `orchestrator` |

---

## 6. DB 상태 전이

허용 상태:
- `in_slot`
- `out`
- `staged`
- `missing`
- `fod_alert`

명령 허용 규칙:

| 명령 | 허용 상태 | 차단 상태 |
|------|-----------|-----------|
| `fetch` | `in_slot` | `out`, `staged`, `missing`, `fod_alert` |
| `return` | `staged` | `in_slot`, `out`, `missing`, `fod_alert` |

FOD 전이:

```text
in_slot --fetch--> out --place_at_staging--> staged
                  |                         |
                  | timeout                 | return/operator pickup
                  v                         v
                missing --alert<=30s--> fod_alert
                                          in_slot
```

---

## 7. 설정 파일

새로 필요하거나 확인할 설정:

```text
config/fod.yaml
config/runtime.yaml
config/toolbox.yaml
config/voice.yaml      # 신규 가능
```

`config/voice.yaml` 후보:

```yaml
schema_version: 1
whisper:
  model: small
  language: ko
  sample_rate_hz: 16000
  temperature: 0.0
vad:
  enabled: true
  threshold: 0.5
wake_word:
  enabled: true
  phrases:
    - "로봇"
    - "두산"
intent:
  min_confidence: 0.7
  db_gate_timeout_s: 0.8
```

좌표, 임계값, timeout은 코드에 하드코딩하지 않고 `config/*.yaml`에서 읽는다.

---

## 8. 테스트 계획

### DB

필수 테스트:
- `fetch` 가능한 상태: `in_slot`
- `fetch` 차단 상태: `out`, `staged`, `missing`, `fod_alert`
- `return` 가능한 상태: `staged`
- `return` 차단 상태: `in_slot`, `out`, `missing`, `fod_alert`
- unknown `tool_id` 실패
- DB 장애 + 캐시 TTL 이내 fallback
- DB 장애 + 캐시 TTL 초과 시 전체 명령 거부
- FOD timeout 전이
- `tool_events` append-only 기록

### Voice

필수 테스트:
- `is_moving=True`이면 STT 호출 안 함
- VAD false이면 Whisper 호출 안 함
- raw text에서 `fetch` intent 생성
- raw text에서 `return` intent 생성
- 모호한 공구명은 `unknown` 또는 확인 요청 처리
- DB Gate 차단이면 실행 intent 발행 금지
- confidence 낮으면 실행 intent 발행 금지

---

## 9. 구현 순서

1. `ros2_ws/src/db` ROS2 Python 패키지 스켈레톤 생성
2. `docs/db-schema.md` 기준으로 DB 접근 모듈을 `ros2_ws/src/db/db/` 내부에 작성
3. `db_service_node.py`에서 `CheckToolFeasibility`, `UpdateToolStatus` 제공
4. FOD monitor 구현
5. DB 서비스 단위 테스트와 서비스 콜백 테스트 작성
6. `ros2_ws/src/voice` ROS2 Python 패키지 스켈레톤 생성
7. `audio_gate.py`와 `whisper_node.py` 구현
8. `gemma_intent_node.py` 구현
9. voice 단위 테스트 작성
10. launch 파일로 DB + voice 노드 실행 경로 정리

---

## 10. 건드리지 않을 범위

다음은 명시 요청 전까지 수정하지 않는다.

- `ros2_ws/src/plc`
- `*_core/`
- `interfaces/` 메시지/서비스 시그니처
- `unit_actions/` 시그니처
- `architecture.html`
- `.claude/rules/*`
- `.claude/agents/*`

특히 `interfaces/` 변경이 필요해 보이면 먼저 문서와 interface-guardian 체크리스트 기준으로 영향 범위를 검토한다.
