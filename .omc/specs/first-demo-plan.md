# 첫 데모 구현 계획 — voice/대시보드 → orchestrator BT → motion 시퀀스

> 브랜치: `feat/demo-runner-v2` (main 기준 rebase 완료)
> 작성일: 2026-06-07 · AI-assisted
> 이전 시도(`feat/demo-runner`)는 별도 demo 패키지 + TCP 미적용으로 실패 → 로컬 보존, 본 계획은 재설계.

---

## 1. 목표

- **시연 범위**: 공구는 `socket_19mm` 하나만 가정. 음성("소켓 가져와/반납해줘") 또는 대시보드 버튼으로 **fetch**(공구함 열기→소켓 파지→Staging 거치→공구함 닫기) / **return**(역순) / **home**.
- vision 미구현 → 공구함·소켓 좌표는 `unit_actions/toolbox_motion.py`에 **하드코딩**(`.tw` 실측값) 사용.
- **orchestrator를 실제로 동작**시켜(현재 전부 스텁) Track A 골격을 시연.
- **대시보드**로 데모뿐 아니라 향후 개발 중 현재 상태를 파악(카메라 2대·공구상태·버튼·E-stop·home·그리퍼 전류 파형·DB/로그 탭). **일부 소스가 죽어도 페이지는 뜨고, 어느 위젯이 오류인지 표시**.
- real(110.120.1.38) / virtual(emulator) **둘 다 지원** (launch 인자 전환).

## 2. 확정된 설계 결정 (2026-06-07)

| # | 결정 | 내용 |
|---|------|------|
| D1 | 오케스트레이션 | **BT + motion 시퀀스**. orchestrator_node가 BT를 실제 tick, fetch/return 리프가 motion 시퀀스 액션을 호출. `unit_action_server` 스텁은 이번엔 건너뜀. |
| D2 | 모션 트리거 | **motion에 영구 시퀀스 액션 서버 추가**(현 `toolbox_seq_runner`는 1회성). 이름으로 시퀀스 반복 실행, TCP 상주. |
| D3 | 대시보드 | **FastAPI + WebSocket + MJPEG + Chart.js** (신규 `dashboard` 패키지). |
| D4 | 실행 대상 | **real/virtual 둘 다**. launch 인자로 전환. |

## 3. 현재 코드 상태 (탐색 결과)

**동작함**: `voice`(whisper+rule_intent→`/voice/intent`, 자체 DB Gate 포함), `db`(db_service_node: CheckToolFeasibility/UpdateToolStatus/LogEvent), `plc`(plc_node, `/plc/system_state`), `motion/toolbox_seq_runner`(로봇 구동 유일·TCP Z+160 설정·그리퍼 서비스, **단 1회성**), `gripper_node`(`/gripper/state` JointState 주기 발행).

**스텁(미구현)**: `orchestrator_node`(intent→blackboard만, **BT tick 안 함**), `fetch_tool`/`return_tool`(Failure 스텁), `unit_action_server`(6액션 전부 abort).

**공백/리스크**:
- ⚠️ `/robot/status`(is_moving)를 **아무도 발행 안 함** → S-7(orchestrator 명령차단 + whisper 오디오 게이팅) 현재 무력. → 시퀀스 서버가 단일 발행자.
- ⚠️ 이전 데모 실패 원인 = **TCP(공구중심점 Z+160) 미적용**. motion 시퀀스 서버가 시작 시 1회 + 시퀀스마다 재확인.
- main의 `socket_fetch_seq`는 **서랍 열기/닫기 미포함** 단순 버전 → 서랍 포함 `full_socket_*` 필요.
- orchestrator BT는 tool_pose를 `/vision/tracked_poses`에서 받게 설계됨 → vision 없으니 하드코딩 시퀀스 경로로 우회.

## 4. 타깃 아키텍처 (데모 흐름)

```
[음성]  whisper_node → rule_intent_node ─┐
                                         ├─(Intent)→ /voice/intent
[대시보드 fetch/return/home 버튼] ───────┘
                                              │
                                   orchestrator_node (BT tick)
                                   ├─ CheckFeasibility 리프 → /db/CheckToolFeasibility (S-2)
                                   ├─ RunSequence 리프  → motion 시퀀스 액션 (fetch/return/home)
                                   ├─ 성공 시 → /db/UpdateToolStatus (out→staged 등)
                                   └─ 상태 → /plc/system_state (LED)
                                              │
                          motion 시퀀스 액션 서버 (신규, 영구)
                          ├─ TCP GripperDA_v1 Z+160 상주
                          ├─ toolbox_motion 시퀀스 실행(move_line/move_joint/gripper)
                          ├─ /robot/status (is_moving) 단일 발행 ← S-7 복구
                          └─ E-stop: dsr motion/move_stop + 래치 정지 + PLC red

[대시보드 백엔드] FastAPI
  ├─ 카메라 워커 ×2 (독립 스레드, 실패해도 다른 카메라/페이지 영향 없음)
  ├─ 구독: /gripper/state(전류 파형), /plc/status, /robot/status, DB 폴링
  ├─ 버튼: fetch/return/home → /voice/intent 주입 / E-stop → 안전 경로
  └─ WebSocket로 위젯별 상태+health 푸시
```

## 5. 패키지 변경 요약

| 패키지 | 변경 | 검토 |
|--------|------|------|
| `interfaces/` | **커스텀 변경 없음** — 기존 `PlaceAtStaging`(=fetch)·`ReturnToSlot`(=return) 액션 재사용. home·E-stop·reset은 표준 `std_srvs/Trigger`. | ✅ 커스텀 인터페이스 무변경 |
| `unit_actions/toolbox_motion.py` | `full_socket_fetch_seq`/`full_socket_return_seq` 추가(서랍 open→grab→staging→close). 기존 시그니처 불변(추가만) | 🔶 interface-guardian(추가 경량 검토) |
| `motion/` | **신규 노드** `tool_action_server.py`: 액션서버 `place_at_staging`(fetch 전체 시퀀스)·`return_to_slot`(return 전체 시퀀스) + 서비스 `~/home`·`~/estop`·`~/estop_reset`(std_srvs/Trigger). TCP 상주·`/robot/status` 발행·E-stop(move_stop+래치). `toolbox_seq_runner`(1회성)는 유지(수동용). ⚠️ 데모에선 stub `unit_action_server` **미기동**(액션명 충돌 방지). | 🔴 safety-reviewer(모션·E-stop·S-7) |
| `orchestrator/` | `orchestrator_node` BT tick 구현; `fetch_tool`=PlaceAtStaging 클라이언트, `return_tool`=ReturnToSlot 클라이언트(CheckFeasibility→액션→UpdateToolStatus); PLC 상태 발행. (home은 대시보드→motion `~/home` 직접, BT 비경유) | 🔴 safety-reviewer(DB Gate·S-7) |

> ⚠️ 기술부채: `PlaceAtStaging`에 "전체 fetch(서랍+슬롯파지+staging)"를 실어 의미 과적재. vision+실제 unit_actions 도입 시 세분 액션으로 재정의할 것.
| `dashboard/`(신규) | FastAPI+uvicorn 백엔드, 정적 웹(카메라2·공구상태·버튼·E-stop·home·전류 파형·DB/로그 탭), 위젯별 health | 일반 |
| `db/`,`plc/`,`voice/` | 변경 없음(그대로 재사용). voice는 fetch/return만; home은 대시보드 버튼 우선(음성 home은 후순위) | — |
| launch | `demo.launch.py`에 real/virtual 토글·plc_port·voice 토글·대시보드 포함 | — |
| `demo/`(신규, ⚠️ 본 계획에 없던 추가) | `demo_trigger`(마이크 없이 `/voice/intent` 1회 publish, DB Gate 우회 없음) + `demo_ui`(8765, `dashboard/`와 별개의 경량 FastAPI 모니터링 — voice/intent·PLC·그리퍼·DB·카메라2). `dashboard/`(8080, D3)을 대체하지 않음 — 독립 보조 도구. → README: `ros2_ws/src/demo/README.md` | 일반 |

## 6. Phase별 작업

### Phase 0 — 준비
- [ ] stash(`feat/demo-runner` 작업)에서 재활용 가능한 자산만 선별 추출: 대시보드 정적 HTML/JS, 카메라 워커(C270 OpenCV+YUYV / RealSense pyrealsense2), `seed_demo_db.py`. (별도 demo 패키지 코드는 폐기)
- [ ] `config/demo.yaml`(tool_id=socket_19mm, db_path) 유지 확인.
- [ ] **udev 규칙** `99-demo-cameras.rules` 추가: C270→`/dev/gripper_cam`, RealSense D455f(UVC color)→`/dev/top_cam` 심볼릭 고정(인덱스 드리프트 차단). `idVendor`/`idProduct`/serial로 매칭. 문서에 `udevadm` 적용 절차 포함.

### Phase 1 — motion tool_action_server (D2) 🔴
- [ ] `unit_actions/toolbox_motion.py`: `full_socket_fetch_seq`/`full_socket_return_seq` 추가(서랍 1층=layer1 기준 open→소켓 파지→`SOCKET_BOTTOM` staging 거치→close→home).
- [ ] `motion/motion/tool_action_server.py` (영구 노드):
  - 액션서버 `place_at_staging`(PlaceAtStaging, goal.tool_id) → `full_socket_fetch_seq` 실행.
  - 액션서버 `return_to_slot`(ReturnToSlot, goal.tool_id) → `full_socket_return_seq` 실행.
  - 서비스 `~/home`(std_srvs/Trigger) → `home_seq` 실행.
  - 시작 시 TCP `GripperDA_v1` Z+160 등록·활성화(서비스 준비될 때까지 재시도) + 각 동작 전 재확인 → **이전 실패 원인 차단**.
  - 실행 전후 `/robot/status` is_moving 발행(S-7 복구). 단일 작성자.
  - `_movel/_movej/_grip`은 `toolbox_seq_runner` 로직 재사용. **executor 데드락 주의**: MTE + 콜백 그룹 분리(액션 실행용 / E-stop용).
  - 서비스 `~/estop`(std_srvs/Trigger, **고우선 콜백 그룹**): dsr `motion/move_stop` 즉시 + 래치 정지(이후 모든 액션/서비스 거부) + `/plc/system_state`=e_stop + DB 로그. 자동 재시작 금지(S-3).
  - 서비스 `~/estop_reset`(std_srvs/Trigger): 운영자 명시적 해제만 래치 해제.
  - 액션 feedback(phase/progress)·result(success/message) 채움.
- [ ] 단위 테스트: 시퀀스 빌드(happy), tool_id 불일치(failure), E-stop 래치 후 액션 거부.

### Phase 2 — 인터페이스 (커스텀 변경 없음)
- [ ] **신규 .action/.srv 없음.** 기존 `PlaceAtStaging`/`ReturnToSlot` 재사용 + `std_srvs/Trigger`(home/estop/reset).
- [ ] `motion/package.xml`에 `std_srvs` 의존 추가.
- [ ] interface-guardian: `unit_actions` 함수 추가만(시그니처 불변) 경량 검토.

### Phase 3 — orchestrator BT 구현 (D1) 🔴
- [ ] `orchestrator_node`: `_on_intent`에서 fetch/return BT를 tick. is_moving(S-7) 가드 유지. (대시보드 fetch/return 버튼도 `/voice/intent` 주입으로 동일 경로.)
- [ ] `bt_nodes/check_feasibility.py`: `/db/CheckToolFeasibility` 호출 리프(S-2).
- [ ] `bt_nodes/run_action.py`(신규): PlaceAtStaging/ReturnToSlot 액션 클라이언트 리프.
- [ ] `fetch_tool`: Sequence[CheckFeasibility(fetch) → PlaceAtStaging(tool_id) → UpdateToolStatus(in_slot→out→staged)]. `return_tool`: Sequence[CheckFeasibility(return) → ReturnToSlot(tool_id) → UpdateToolStatus(staged→out→in_slot)].
- [ ] 성공/실패 → `/plc/system_state`(moving/idle/error) 발행.
- [ ] 골든/단위 테스트: BT 조립, 가짜 액션서버로 fetch happy/feasibility-blocked.

### Phase 4 — dashboard 패키지 (D3)
- [ ] `dashboard/` ament_python 패키지(+ `setup.cfg`로 console_scripts → `lib/dashboard/`).
- [ ] 백엔드(FastAPI+uvicorn): 
  - 카메라 워커 2개 **독립 스레드**(하나 실패해도 다른 하나·페이지 정상). 카메라는 `/dev/video*` 열거 후 **이름 매칭**(인덱스 드리프트 대비). 실패 시 placeholder + "CAM 오류" 오버레이.
  - 구독: `/gripper/state`(전류=effort) 파형, `/plc/status`, `/robot/status`; DB는 주기 폴링.
  - 버튼: fetch/return → `/voice/intent` Intent 주입(raw_utterance="dashboard", orchestrator BT 경유). home → motion `~/home`(Trigger). E-stop → motion `~/estop`(Trigger), reset → `~/estop_reset`. 
  - 위젯별 `health: ok|error|stale` 포함해 WebSocket 푸시 → **부분 장애 시각화**.
  - 탭: ① 실시간(카메라·상태·버튼·파형) ② DB 상태 ③ 로그(tool_events/system_events).
- [ ] 프론트(정적): GitHub 다크 테마, Chart.js 파형, WebSocket 자동 재연결, 탭바.

### Phase 5 — launch 통합 (D4)
- [ ] `demo.launch.py`: 인자 `mode:=real|virtual`, `robot_ip:=110.120.1.38`, `voice:=`, `plc:=`, `plc_port:=`, `dashboard:=`.
- [ ] real: doosan bringup + 실 카메라/PLC. virtual: emulator + 카메라 없는 위젯은 error 표시.

### Phase 6 — 검증
- [ ] emulator에서 fetch/return/home BT 경로 + DB 상태 전이 + 대시보드 위젯 확인.
- [ ] DB Gate: staged 상태에서 fetch 거부, in_slot 아닐 때 거부 로그.
- [ ] 카메라 0/1/2대 연결 각각에서 페이지 정상 로드 + 오류 표시.
- [ ] **실물(110.120.1.38)**: TCP 적용 확인(첫 동작이 좌표와 일치), E-stop ≤500ms 측정, S-7(이동 중 명령 무시) 확인.
- [ ] 매 테스트 전 `python3 scripts/seed_demo_db.py`로 in_slot 리셋.

## 7. 안전 — 사인오프 필요 (제안 기본값)

- **E-stop**(S-3, 확정: move_stop+래치): 대시보드 버튼 → motion `~/estop`(std_srvs/Trigger, 고우선 콜백 그룹) → ① dsr `motion/move_stop` 즉시 ② 래치 정지(신규 액션/서비스 거부) ③ PLC red solid ④ DB 로그 ⑤ 자동 재시작 금지, `~/estop_reset`로만 해제. 목표 ≤500ms. **음성 E-stop("멈춰")은 v1.0 미구현(S-7, 오인식 위험)** → 버튼/물리만.
- **DB Gate**(S-2): 대시보드 버튼도 orchestrator BT의 CheckFeasibility를 반드시 통과(주입 intent도 동일 경로).
- **S-7**: 시퀀스 서버가 `/robot/status` 단일 발행 → orchestrator·whisper 게이팅 복구.
- 모션/E-stop/orchestrator 변경은 **safety-reviewer**, interfaces/unit_actions는 **interface-guardian** 검토 후 머지(CLAUDE.md).

## 8. 확정된 결정 (2026-06-07)

1. ✅ **E-stop 깊이**: move_stop(감속 정지) + 래치. servo-off는 미적용.
2. ✅ **카메라 식별**: udev 별칭(`/dev/gripper_cam`, `/dev/top_cam`) — Phase 0에서 규칙 추가.
3. ✅ **home**: 대시보드 버튼만(motion `~/home` Trigger). 음성 home 미구현.
4. ✅ **인터페이스**: 커스텀 변경 없음. fetch=`PlaceAtStaging`, return=`ReturnToSlot` 재사용 + home/estop/reset=`std_srvs/Trigger`.
   - ⚠️ 잔여 기술부채: `PlaceAtStaging`에 전체 fetch 의미 과적재 → vision+unit_actions 도입 시 세분 액션으로 재정의.
