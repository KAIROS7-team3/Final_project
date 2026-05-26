# Voice-Commanded Tool Delivery Robot Arm

> 음성 명령으로 공구함에서 공구를 꺼내 전달하고 반납 시 제자리에 돌려놓는 협동 로봇팔 시스템.
> **현재 단계:** 설계 완료 — 코드 구현 시작 전 (Phase 0 진입 준비 중).

---

## 한 줄 요약

Doosan e0509 협동 로봇팔이 운영자의 음성 명령을 듣고 공구함의 9종 공구를 Staging Area에 거치/회수하는 시스템. 세 가지 제어 방식(BT+DSR, BT+RL, end-to-end VLA)을 동일 하드웨어에서 비교 평가한다.

## 누구를 위한 저장소인가

- **연구**: 구조화된 BT+unit_actions vs end-to-end 학습 기반 제어 비교
- **운영**: 실험실/소규모 정비소에서 공구 관리 자동화
- **교육**: 협동 로봇 + 비전 + LLM + 안전 시스템 통합 사례

---

## Quick Start

### 처음 클론한 팀원

| 순서 | 작업 | 상세 |
|------|------|------|
| 1 | AI 도구 설정 | [`docs/ai-setup.md`](docs/ai-setup.md) — Claude Code/Codex/Cursor 비교 + 설정 |
| 2 | 프로젝트 개요 파악 | [`CLAUDE.md`](CLAUDE.md) — 아키텍처 한눈에 보기 |
| 3 | Phase 일정 확인 | [`robot-arm-project.md`](robot-arm-project.md) — Phase 0–9 로드맵 |
| 4 | 환경 변수 설정 | `cp .env.example .env` → 값 채우기 (팀장에게 문의) |
| 5 | 룰 숙지 | [`.claude/rules/`](.claude/rules/) — safety/engineering/process |

### 실행 (코드 구현 후)

```bash
./run.sh --track A          # Gemma 4 + BT + DSR
./run.sh --track B          # Gemma 4 + BT + RL
./run.sh --track C          # 키워드 파서 + VLA (no ROS2)
./run.sh --track A --sim    # 시뮬레이션
./run.sh --help             # 전체 옵션
```

---

## 시스템 구성

### 하드웨어

| 구성요소 | 모델 |
|----------|------|
| 로봇팔 | Doosan e0509 (6-DOF 협동) |
| 그리퍼 | ROBOTIS RH-P12-RN |
| 카메라 | Intel RealSense D455f (eye-in-hand) |
| PLC | LS Electric XBC-DR10E |
| 메인 PC | Vector 16 HX AI A2XWIG |

상세 → [`docs/hardware.md`](docs/hardware.md)

### 소프트웨어 스택

| 레이어 | Track A/B | Track C |
|--------|-----------|---------|
| OS | Ubuntu 22.04 | 동일 |
| 미들웨어 | ROS2 Humble | (없음 — 단일 Python) |
| STT | Whisper small | 동일 |
| 의도 분류 | Gemma 4 7B (로컬) | Python 키워드 파서 |
| 모션 | DSR 좌표 제어 (A) / RL 정책 (B) | VLA 모델 end-to-end |
| 비전 | YOLOv8 + 6D pose | VLA 입력 (raw RGB-D) |
| 하드웨어 인터페이스 | `doosan-robot2` ROS2 드라이버 | Doosan Python SDK 직접 |

---

## 저장소 구조

```
.
├── README.md                  ← 이 파일
├── CLAUDE.md                  ← AI 에이전트 + 팀원용 프로젝트 개요
├── robot-arm-project.md       ← Phase 0–9 개발 계획
├── run.sh                     ← Track Selector
├── architecture.html          ← 인터랙티브 시각화
├── docs/                      ← 모든 설계 문서
│   ├── architecture.md            아키텍처 + 패키지 구조
│   ├── adr/                       ADR (카테고리별) + 미결 사항 — index.md로 시작
│   ├── conventions.md             네이밍 + 수락 기준
│   ├── interfaces.md              msg/srv/action 계약
│   ├── frames.md                  좌표계 + TF tree
│   ├── hardware.md                하드웨어 인벤토리
│   ├── simulation.md              Gazebo 골든 파일 회귀
│   ├── db-schema.md               DB 논리 스키마
│   ├── ai-setup.md                AI 도구 설정 안내
│   ├── logging.md                 로깅 인프라 (파일 위치, 순환, DB 이벤트)
│   └── glossary.md                용어 사전
├── config/                    ← 비-시크릿 운영 파라미터
│   ├── runtime.yaml               robot_model, whisper_size 등
│   ├── staging_area.yaml          공구별 거치 좌표
│   ├── toolbox.yaml               슬롯 + 공구 카탈로그
│   ├── hand_eye.yaml              카메라-EE 변환
│   ├── robot_poses.yaml           home/scan 포즈
│   └── fod.yaml                   FOD 임계
├── interfaces/                ← ROS2 msg/srv/action (코드 구현 시)
│   └── CHANGELOG.md
├── .env.example               ← 시크릿 변수 템플릿 (값 없음)
├── .claude/                   ← AI 에이전트 설정 (Claude Code 기준)
│   ├── rules/                     safety/engineering/process 룰
│   ├── agents/                    robot-arm-planner/safety-reviewer/interface-guardian
│   ├── skills/                    프로젝트 전문 지식 (20여 종)
│   └── settings.json              권한 + hooks
└── .omc/specs/                ← 원본 설계 문서
```

---

## 개발 단계

| Phase | 내용 | 상태 |
|-------|------|------|
| 0 | 인프라 + 인터페이스 + 캘리브레이션 | 진입 준비 |
| 1 | HAL + unit_actions | 대기 |
| 2 | 비전 (YOLOv8 + pose) | 대기 |
| 3 | DB + PLC | 대기 |
| 4 | 음성 (Whisper + Gemma 4) | 대기 |
| 5 | Track A BT 통합 + Track B RL | 대기 |
| 6 | Track C VLA 파인튜닝 + 통합 | 대기 |
| 7 | 통합 테스트 + 트랙 비교 | 대기 |
| 8 | 운영 배포 + 모니터링 | 대기 |
| 9 | v1.0 릴리스 + 운영 | 대기 |

상세 → [`robot-arm-project.md`](robot-arm-project.md)

---

## 기여 가이드

상세 → [`CONTRIBUTING.md`](CONTRIBUTING.md)

1. **룰 숙지 필수** — [`.claude/rules/`](.claude/rules/) (safety > engineering > process)
2. **PR 전 검토** — 안전 코드는 `safety-reviewer`, 공유 인터페이스 변경은 `interface-guardian` 에이전트
3. **커밋 형식** — Conventional Commits 변형, [`.claude/rules/process.md`](.claude/rules/process.md) P-4
4. **테스트** — 안전 critical 경로는 테스트 없으면 머지 금지 (P-1)

---

## 라이선스

TBD — 프로젝트 이름·원격 저장소 확정 후 결정.
