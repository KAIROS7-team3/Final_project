# DB 논리 스키마

> 이 문서는 논리 스키마를 정의한다. DB 엔진: **SQLite WAL 모드** (ADR-008).
> 물리 DDL은 `db_core/schema.sql`, 마이그레이션은 `db_core/migrations/`에 작성.
> 참조: `docs/architecture.md`, `.claude/rules/safety.md` S-2/S-8, `.claude/rules/engineering.md` E-6

---

## 1. 테이블 개요

| 테이블 | 용도 | 카디널리티 |
|--------|------|------------|
| `tools` | 공구 카탈로그 + 현재 상태 (단일 진실) | 9 rows (v1.0 고정) |
| `tool_events` | 모든 fetch/return/error/rejected 이벤트 로그 (append-only) | 운영 누적 |
| `operators` | 운영자 카탈로그 | v1.0 단일 row (결정 #20) |
| `system_events` | 부팅·reconciliation·E-stop 등 시스템 이벤트 로그 | 운영 누적 |

---

## 2. `tools` — 공구 카탈로그 + 현재 상태

**역할**: 각 공구의 현재 위치/상태에 대한 **단일 진실(single source of truth)**.

| 컬럼 | 타입 | 제약 | 의미 |
|------|------|------|------|
| `tool_id` | TEXT | PRIMARY KEY | `^[a-z][a-z0-9]*(_[a-z0-9]+)+$` (interfaces.md §0) |
| `display_name` | TEXT | NOT NULL | 사용자에게 표시할 한글 이름 (예: "필립스 드라이버 (소)") |
| `current_status` | TEXT | NOT NULL CHECK | enum: `in_slot` \| `out` \| `staged` \| `missing` \| `fod_alert` |
| `home_slot_row` | INTEGER | NOT NULL | 원래 슬롯 행 (toolbox.yaml과 일치) |
| `home_slot_col` | INTEGER | NOT NULL | 원래 슬롯 열 |
| `last_event_id` | INTEGER | FK → `tool_events.event_id` | 가장 최근 이벤트 (감사 추적) |
| `last_updated` | TIMESTAMP | NOT NULL | 마지막 상태 변경 시각 (UTC) |

**불변 조건 (invariants):**
- `current_status`는 trigger 또는 애플리케이션 로직으로 `tool_events`의 가장 최근 결과와 항상 일치
- `home_slot_*`는 v1.0에서 변경 금지 (toolbox 재배치 시에만 갱신)

---

## 3. `tool_events` — 이벤트 로그 (append-only)

**역할**: 모든 공구 관련 이벤트의 불변 기록. DB Gate(S-2), FOD 감지(S-8), 디버깅·분석에 사용.

| 컬럼 | 타입 | 제약 | 의미 |
|------|------|------|------|
| `event_id` | INTEGER | PRIMARY KEY AUTOINCREMENT | |
| `tool_id` | TEXT | NOT NULL FK → `tools.tool_id` | |
| `event_type` | TEXT | NOT NULL CHECK | enum (§3.1 참조) |
| `track` | TEXT | NOT NULL CHECK | `A` \| `B` \| `C` |
| `operator_id` | TEXT | NOT NULL FK → `operators.operator_id` | v1.0은 `operator_01` 고정 (결정 #20) |
| `status_before` | TEXT | NULL | 이벤트 직전 `tools.current_status` (스냅샷) |
| `status_after` | TEXT | NOT NULL | 이벤트 직후 `tools.current_status` |
| `notes` | TEXT | NULL | 자유 텍스트 (에러 메시지, 거부 사유 등) |
| `timestamp` | TIMESTAMP | NOT NULL DEFAULT NOW | 이벤트 발생 시각 (UTC) |

**불변 조건:**
- **Append-only**: UPDATE/DELETE 금지 (애플리케이션 레이어에서 차단, DB 권한으로도 가능)
- `timestamp`는 monotonic 보장 안 됨 (NTP 보정 가능). `event_id` 순서가 정확한 순서

### 3.1 `event_type` enum

| 값 | 의미 | 발생 조건 |
|----|------|----------|
| `fetch` | 공구 꺼내기 완료 | 슬롯 → staging 거치 완료 |
| `return` | 공구 반납 완료 | staging/외부 → 슬롯 |
| `rejected` | DB Gate에서 차단된 명령 | 불가 명령 (S-2): missing/out/fod_alert 상태 공구 fetch 등 |
| `error` | 하드웨어/소프트웨어 실패 | 모션/그리퍼/PLC 실패 (E-5) |
| `timeout` | checkout 시간 초과 → `missing` 전이 (경보 전 단계) | 자동 (S-8): `out`/`staged` → `missing` |
| `fod_alert` | FOD 임계(grace) 초과 → 분실 경보 | 자동 (S-8): `missing` → `fod_alert`, `system_events`에도 critical 기록 (E-5) |
| `reconciled` | 부팅 시 YOLOv11s 스캔으로 상태 동기화 | 부팅 시 1회 (S-9) |

### 3.2 인덱스 권장

- `(tool_id, timestamp DESC)` — 특정 공구의 최근 이벤트 조회
- `(event_type, timestamp DESC)` — 에러/거부 이벤트 집계
- `(track, timestamp DESC)` — 트랙별 분석

---

## 4. `operators` — 운영자 카탈로그

**역할**: v1.0에서는 단일 운영자(결정 #20). v2.0+에서 음성 화자 식별 시 확장.

| 컬럼 | 타입 | 제약 | 의미 |
|------|------|------|------|
| `operator_id` | TEXT | PRIMARY KEY | `^[a-z][a-z0-9_]*$` |
| `display_name` | TEXT | NOT NULL | 표시용 이름 |
| `created_at` | TIMESTAMP | NOT NULL | 등록 시각 |

**v1.0 seed:**
```
operator_id  = 'operator_01'
display_name = 'default operator'
```

---

## 5. `system_events` — 시스템 이벤트 로그

**역할**: 부팅, reconciliation, E-stop, 캘리브레이션 등 비-공구 이벤트.

| 컬럼 | 타입 | 제약 | 의미 |
|------|------|------|------|
| `event_id` | INTEGER | PRIMARY KEY AUTOINCREMENT | |
| `event_type` | TEXT | NOT NULL CHECK | enum (§5.1) |
| `track` | TEXT | NULL | 해당하는 경우 (`A`/`B`/`C`), 시스템 전역이면 NULL |
| `severity` | TEXT | NOT NULL CHECK | `info` \| `warning` \| `error` \| `critical` |
| `notes` | TEXT | NULL | 상세 |
| `timestamp` | TIMESTAMP | NOT NULL | UTC |

### 5.1 `event_type` enum (system)

| 값 | 의미 |
|----|------|
| `boot` | 시스템 부팅 시작 |
| `boot_complete` | reconciliation 완료 + 명령 수신 가능 |
| `reconciliation_mismatch` | 부팅 시 DB vs YOLOv11s 불일치 (운영자 확인 필요, S-9) |
| `estop` | E-stop 트리거 (S-3) |
| `estop_reset` | E-stop 해제 후 정상 복귀 |
| `db_cache_fallback` | DB 연결 실패 → 캐시 사용 (S-2, 결정 #12) |
| `db_cache_expired` | 캐시 TTL 초과 → 모든 명령 거부 (S-2) |
| `calibration` | 캘리브레이션 시작/완료 |
| `fod_alert` | FOD 경보(`tool_events`의 `missing → fod_alert`와 동반) | 자동 (S-8/E-5), severity=`critical` |

---

## 6. 무결성 규칙

1. **`tools.current_status`와 마지막 `tool_events.status_after`는 항상 일치** — 애플리케이션 또는 trigger로 강제
2. **`tool_events`는 append-only** — UPDATE/DELETE 금지
3. **모든 타임스탬프는 UTC** — 표시 시점에 KST 변환
4. **`operator_id`는 v1.0에서 `operator_01` 고정** — 변경 시 ADR 추가 필요
5. **DB 연결 실패 시 캐시 TTL 5분** (결정 #12, S-2) — 캐시 정책은 `db_core/`에서 구현

---

## 7. 마이그레이션 정책

- 스키마 변경은 `db_core/migrations/` 디렉토리에 순차 번호 SQL 파일로 관리
- 마이그레이션은 idempotent해야 함 (재실행 안전)
- 운영 DB의 백업은 마이그레이션 직전 자동 (Phase 7+)
- 스키마 변경 PR은 `interface-guardian` 검토 필수 (db_core 변경이므로)

---

## 8. 미결 / 후속 작업

| 항목 | 결정 시점 |
|------|----------|
| 백업/복구 정책 | Phase 7 |
| 데이터 보존 기간 (이벤트 로그 archival) | Phase 7 |
| 분석용 read replica 필요 여부 | Phase 8+ |
