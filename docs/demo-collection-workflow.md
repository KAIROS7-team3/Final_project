# Track C VLA — Demonstration 수집 워크플로우

> Track C VLA 모델 파인튜닝용 demonstration 데이터를 수집하는 전체 절차.
> **본격 수집**: Phase 4 안정화 후(G4 통과 후) 시작. 옵션 A(Track A 활용)는 Phase 5a 종료 후 가능, 옵션 B/C는 Phase 4 종료 후 즉시 가능.
> 약 **900 demo / 7.5h 순수 녹화 / 20h+ 실패 포함**.
> 참조: `docs/adr/ai-ml.md` ADR-004, 미결 #5 (VLA 모델 선정), `.claude/skills/demo-collection/SKILL.md`

---

## 1. 목표 데이터

### 1.1 데이터셋 명세

| 항목 | 값 |
|------|-----|
| 공구 종류 | 9종 (`config/toolbox.yaml`) |
| 동작 | `fetch` (슬롯 → staging) + `return` (staging → 슬롯) |
| demo / (공구 × 동작) | 50개 |
| **총 demo** | **9 × 2 × 50 = 900** |
| 실패/재시도 포함 예상 시간 | 20h+ |
| 순수 녹화 시간 (한 demo ~30초 가정) | ~7.5h |

### 1.2 각 demo가 담아야 하는 것

| 채널 | 형식 | 출처 | 샘플링 |
|------|------|------|--------|
| RGB 이미지 | PNG/JPEG (640×480 권장) | RealSense D455f color stream | 15 Hz 이상 |
| Depth 이미지 | 16-bit PNG (mm) | RealSense D455f depth stream | RGB와 시각 동기화 |
| Joint state | 6-DOF position + velocity | Doosan Python SDK | 30 Hz 이상 |
| Gripper command | 연속값 (0.0=open ~ 1.0=close) | 로봇 명령 로그 | 동작 발생 시 |
| TCP pose | (x, y, z, qx, qy, qz, qw) base_link 기준 | Doosan SDK FK | 30 Hz 이상 |
| Language instruction | 한국어 문장 | 운영자 입력 | 1회 / demo |
| Timestamp | UTC microsecond | OS clock | 모든 채널 |
| Success flag | bool | 운영자 판정 | 1회 / demo (녹화 종료 시) |
| Tool ID | 문자열 (`config/toolbox.yaml`) | 운영자 입력 | 1회 / demo |
| Trial metadata | YAML | 자동 생성 | 1회 / demo |

---

## 2. 수집 방식

### 2.1 옵션 비교

| 방식 | 설명 | 장점 | 단점 |
|------|------|------|------|
| **A. Track A 활용** | Track A(BT+DSR)로 자동 실행, 결과 녹화 | 자동화 가능, 일관된 품질 | Track A 코드 완성 의존 (Phase 5a 후) |
| **B. Kinesthetic teaching** | 로봇 free-drive 모드 → 사람이 직접 움직임 | 코드 의존 없음, 다양한 trajectory | 운영자 부담, e0509 free-drive 학습 필요 |
| **C. Teleoperation** | 조이스틱/SpaceMouse로 원격 조작 | 안전, 다양성 | 입력 장치 + 매핑 코드 필요 |

→ **권장: A (Track A 활용)를 기본, 다양성 확보용으로 B 일부 병행**. A로 7할, B로 3할 수집 → 도메인 일반화 향상.

### 2.2 환경 통제

- **조명**: 실험실 표준 조명 (수락 기준 §공통과 동일 조건)
- **공구 배치**: 매 demo 시작 전 `config/toolbox.yaml`에 따른 정렬 (reconciliation 통과 상태)
- **방해 요소**: 사람·외부 물체 카메라 시야 차단 (단, 일부 demo에 의도적 변화 포함 — §3.4)

---

## 3. 데이터 다양성 전략

같은 (공구, 동작) 조합 50개가 모두 동일하면 모델이 과적합한다.
**의도적 변화**를 분배:

| 변화 차원 | 변동 범위 | demo 비율 |
|-----------|-----------|-----------|
| 시작 joint pose | home ± 작은 perturbation | 100% (자연 변동) |
| 공구 슬롯 미세 위치 | ±5mm | 30% |
| 조명 (밝기) | 표준 100% / 70% / 130% | 각 30% / 50% / 20% |
| Language instruction 변형 | 같은 의미 5가지 표현 | 균등 분배 |
| 카메라 노이즈 | 자연 | 100% |
| **방해물 (사람 손, 다른 공구)** | 시야 일부 가림 | 10% (강건성용) |

Language instruction 예시 (`fetch screwdriver_phillips_small`):
- "필립스 드라이버 작은 거 갖다 줘"
- "작은 십자 드라이버 꺼내 줘"
- "필립스 소형 좀"
- "스크류드라이버 필립스 스몰 가져와"
- "공구함에서 필립스 드라이버 작은 거"

---

## 4. 운영 절차

### 4.1 1회 demo 녹화

```
1. 시작 조건 확인
   - 로봇 home pose
   - 공구함 reconciled 상태
   - DB system_events에 'demo_session_start' 기록
   - Staging Area 비어있음 (fetch demo) / 공구 staged (return demo)

2. demo recorder 시작 (가칭: scripts/record_demo.py)
   - 매개변수: --tool_id <id> --action {fetch|return} --instruction "<발화>"
   - 모든 채널 녹화 시작, 자동 timestamp + 폴더 생성

3. 운영자가 해당 동작 수행 (방식 A/B/C 중 하나)

4. 동작 완료 후
   - 운영자가 success/failure 판정 (terminal prompt)
   - 실패면 사유 메모 (예: "그리퍼 미끄러짐", "approach 각도 어긋남")
   - recorder가 metadata.yaml + manifest.jsonl에 기록 → 종료

5. 환경 리셋 (다음 demo 시작 조건)
```

### 4.2 세션 단위 운영

- 1 세션 = ~50 demo (~30분 + 환경 리셋 포함 ~1.5h)
- 세션 시작/종료 시 reconciliation 1회
- 세션마다 hand_eye 캘리브레이션 재검증 (3 marker 측정, drift 확인)
- 1일 최대 4 세션 권장 (운영자 피로도)

---

## 5. 저장 / 명명 규칙

### 5.1 디렉토리 구조

```
data/track_c_demos/
├── manifest.jsonl                      ← 전체 demo 인덱스 (append-only)
├── sessions/
│   └── 2026-06-15_morning_session01/
│       ├── session_meta.yaml           ← 세션 환경 (조명, 운영자, 캘리브 결과)
│       └── demos/
│           └── 20260615T093215_fetch_screwdriver_phillips_small_001/
│               ├── meta.yaml           ← demo 메타데이터 (instruction, success, notes)
│               ├── rgb/000001.jpg ...
│               ├── depth/000001.png ...
│               ├── joints.parquet      ← joint state 시계열
│               ├── tcp.parquet         ← TCP pose 시계열
│               └── gripper.parquet     ← gripper command 시계열
```

### 5.2 명명 규약

| 항목 | 형식 |
|------|------|
| Session 이름 | `YYYY-MM-DD_{morning|afternoon|evening}_session{NN}` |
| Demo 이름 | `YYYYMMDDTHHMMSS_{action}_{tool_id}_{NNN}` |
| Frame index | 6자리 zero-padded (000001 ~) |
| Time format | ISO 8601 (UTC, `Z` suffix) |

### 5.3 manifest.jsonl 한 줄 예시

```json
{"demo_id": "20260615T093215_fetch_screwdriver_phillips_small_001", "session": "2026-06-15_morning_session01", "tool_id": "screwdriver_phillips_small", "action": "fetch", "instruction": "필립스 드라이버 작은 거 갖다 줘", "success": true, "duration_s": 28.4, "n_frames": 426, "collection_method": "A", "lighting": "standard", "operator_id": "operator_01", "checksum": "sha256:..."}
```

---

## 6. 품질 기준 (Acceptance)

### 6.1 demo 단위 통과 기준 (success=true 조건)

- [ ] 시작 조건이 명시된 상태와 일치
- [ ] 목표 공구가 의도된 위치(staging or slot)에 도달 (`±5mm`)
- [ ] 그리퍼가 공구를 떨어뜨리지 않음
- [ ] 충돌·E-stop 발생하지 않음
- [ ] 모든 채널 동기화 OK (timestamp drift ≤ 50ms)
- [ ] 모든 frame 존재 (depth missing 0개)

### 6.2 데이터셋 단위 통과 기준 (학습 진입 전)

- [ ] 900 demo 모두 success=true
- [ ] (공구, 동작) 조합당 50개 정확히 보유
- [ ] §3 다양성 비율 ±5% 이내 준수
- [ ] manifest.jsonl과 실제 파일 시스템 무결성 일치 (checksum 검증)
- [ ] 학습용 8 : val 1 : test 1 split 정의 + 균등 (공구별 5 : 0.5 : 0.5)

### 6.3 실패 처리

- 실패 demo도 `manifest.jsonl`에 `success: false`로 기록 (디버깅·실패 분석용 보존)
- 학습용 데이터셋 생성 스크립트(`scripts/build_dataset.py`)가 success=true만 필터
- 실패율 > 30% 시 세션 중단 후 캘리브레이션·하드웨어 점검

---

## 7. 필요한 도구 / 스크립트

| 스크립트 | 역할 | 작성 시점 |
|----------|------|----------|
| `scripts/record_demo.py` | 1회 demo 녹화 (모든 채널 동기) | Phase 4 종료 |
| `scripts/list_demos.py` | manifest.jsonl 조회·필터 | Phase 4 종료 |
| `scripts/validate_demo.py` | §6.1 demo 단위 검증 | Phase 4 종료 |
| `scripts/build_dataset.py` | manifest → 학습용 split (HF Dataset / RLDS 형식) | Phase 6 시작 |
| `scripts/replay_demo.py` | 녹화된 trajectory를 시뮬레이션에서 재생 (검증용) | Phase 6 시작 |

→ 미결 #5 (VLA 모델) 선정 결과에 따라 `build_dataset.py` 출력 형식 결정 (OpenVLA는 RLDS, π0은 LeRobot 형식 등).

---

## 8. 저장 용량 / 백업

### 추정 용량

- RGB 1 frame: ~100 KB (JPEG q90, 640×480)
- Depth 1 frame: ~600 KB (16-bit PNG)
- 1 demo (~30s × 15 Hz): ~450 frames × 700 KB ≈ **315 MB**
- **900 demo 총량 ≈ 280 GB**

### 백업 정책

- 1차: Vector 16 HX 로컬 NVMe (수집 즉시)
- 2차: NAS / 외장 디스크 (세션 종료 시 매일)
- 3차: 클라우드 (Phase 6 시작 전 일괄, S3/GCS — 미결)
- checksum (SHA-256) 모두 manifest에 기록 → 무결성 검증

---

## 9. 안전 고려 (운영자)

- Kinesthetic teaching 시 free-drive 모드 → 의도치 않은 가속 가능, **충돌 영역 확보 필수**
- 운영자 피로 누적 시 demo 품질 저하 → §4.2 1일 4세션 제한 준수
- E-stop 발생 시 해당 demo는 자동 실패 처리 + 시스템 점검 후 재개

---

## 10. 미결 / 후속 작업

| 항목 | 결정 시점 |
|------|----------|
| VLA 모델 선정 → 데이터 출력 형식 (RLDS / LeRobot / 자체) | Phase 6 시작 전 (미결 #5) |
| 클라우드 백업 위치 (AWS S3 / GCS / Lambda Labs) | Phase 6 시작 전 |
| Kinesthetic teaching vs Teleoperation 비율 | 첫 100 demo 수집 후 검토 |
| Language instruction 다국어 (영어 추가?) | v2.0+ |
| Demo 재사용 정책 (Track A 학습용으로도 활용?) | 데이터 수집 완료 후 |
