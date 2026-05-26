# `.claude/skills/` 카탈로그

> 이 프로젝트의 도메인 지식 스킬 25종. Claude Code가 키워드 자동 트리거로 호출한다.
> Codex/Gemini 사용자는 [`AGENTS.md`](../../AGENTS.md)의 §7 표 참조 후 필요 시 직접 읽는다.

---

## 포맷

모든 스킬은 디렉토리 형식이다:

```
.claude/skills/<skill-name>/
├── SKILL.md          # 자동 로드 대상 (필수)
└── REFERENCE.md      # 원본 전체 버전 (슬림화된 스킬에만 존재)
```

`SKILL.md`의 frontmatter는 `name`(디렉토리명과 동일) + `description`(자동 로드되어 트리거 매칭)을 포함한다.

---

## 카테고리별 목록

### 프로젝트 전용 (8종) — 본 프로젝트 직접 작성

| 스킬 | 설명 |
|------|------|
| [`doosan-e0509`](doosan-e0509/SKILL.md) | Doosan e0509 사양·관절 한계·DSR/SDK |
| [`modbus-plc`](modbus-plc/SKILL.md) | PLC Modbus 통신·LED 매핑 |
| [`whisper-stt`](whisper-stt/SKILL.md) | Whisper STT·한국어·VAD·오인식 방지 |
| [`vla-finetuning`](vla-finetuning/SKILL.md) | OpenVLA/π0 파인튜닝·Q4 양자화 |
| [`demo-collection`](demo-collection/SKILL.md) | Demonstration 수집·RLDS 변환 |
| [`bt-py-trees`](bt-py-trees/SKILL.md) | py_trees Behavior Tree·골든 파일 |
| [`realsense-d455f`](realsense-d455f/SKILL.md) | D455f·pyrealsense2·realsense-ros |
| [`hand-eye-calibration`](hand-eye-calibration/SKILL.md) | D455f+e0509 핸드-아이 캘리브레이션 |

**출처**: 본 프로젝트 자체 작성. 외부 라이브러리(`doosan-robot2`, `easy_handeye2`, `pyrealsense2` 등)의 GitHub 링크를 본문 §참고 절에 명시.

---

### 공통 패턴 (7종) — 본 프로젝트 직접 작성

| 스킬 | 설명 |
|------|------|
| [`python-patterns`](python-patterns/SKILL.md) | 현대 Python 3.10+ 패턴 |
| [`error-handling-patterns`](error-handling-patterns/SKILL.md) | 예외 계층·retry·circuit breaker |
| [`config-management`](config-management/SKILL.md) | YAML 스키마·pydantic·`.env` |
| [`pytest-patterns`](pytest-patterns/SKILL.md) | fixture·parametrize·ROS2 mock |
| [`performance-profiling`](performance-profiling/SKILL.md) | cProfile·VRAM·async 병목 |
| [`code-review-checklist`](code-review-checklist/SKILL.md) | PR 리뷰 체크리스트 |
| [`git-conventions`](git-conventions/SKILL.md) | Conventional Commits·브랜치 |

**출처**: 본 프로젝트 자체 작성. 본문에 외부 도구(ruff, mypy, pydantic, pytest 등) 참고 링크 포함.

---

### 일반 robotics 참조 (10종) — 외부 컬렉션 수입

| 스킬 | 라인 수 | 슬림 여부 |
|------|---------|----------|
| [`ros2`](ros2/SKILL.md) | 994 | 원본 유지 (자주 참조) |
| [`ros1`](ros1/SKILL.md) | ~80 | ✂️ 슬림 — 원본은 `ros1/REFERENCE.md` |
| [`robot-bringup`](robot-bringup/SKILL.md) | 1806 | 원본 유지 (Phase 9 배포 시 필요) |
| [`robot-perception`](robot-perception/SKILL.md) | 1654 | 원본 유지 (`realsense-d455f`로 부족한 일반 패턴 보완) |
| [`robotics-design-patterns`](robotics-design-patterns/SKILL.md) | 609 | 원본 유지 (`robot-arm-planner` 에이전트 의존) |
| [`robotics-software-principles`](robotics-software-principles/SKILL.md) | 896 | 원본 유지 |
| [`robotics-security`](robotics-security/SKILL.md) | 890 | 원본 유지 (Phase 9 SROS2 도입 시) |
| [`robotics-testing`](robotics-testing/SKILL.md) | 577 | 원본 유지 (`pytest-patterns` 보완) |
| [`docker-ros2-development`](docker-ros2-development/SKILL.md) | 1102 | 원본 유지 (Phase 9 컨테이너화 시) |
| [`ros2-web-integration`](ros2-web-integration/SKILL.md) | ~80 | ✂️ 슬림 — 원본은 `ros2-web-integration/REFERENCE.md` |

**출처**: <https://github.com/arpitg1304/robotics-agent-skills>
원본 전체 버전이 필요하면 위 저장소 또는 슬림화된 스킬의 동일 디렉토리 `REFERENCE.md` 참조.

---

## 슬림화 정책

자동 로드되는 description의 길이가 매 세션마다 토큰 비용. 따라서:

1. **본문이 1000+ 라인**이고 **본 프로젝트와 직접 무관**한 스킬은 슬림화 대상
2. 슬림 방식: 원본을 같은 디렉토리의 `REFERENCE.md`로 이동 → 새 `SKILL.md`는 80~150줄로 작성
3. 새 `SKILL.md`는 다음 구조:
   ```markdown
   ---
   name: <name>
   description: <one-line>
   ---
   # <Title>
   > 본 프로젝트에서는 X 용도로만 사용. 전체 참고는 [`REFERENCE.md`](REFERENCE.md).

   ## 본 프로젝트 핵심 사용 사례
   ...

   ## 본 프로젝트가 사용하지 않는 영역
   - ... → 필요 시 REFERENCE.md
   ```
4. **본 프로젝트와 자주 닿는 스킬**(ros2, robot-bringup 등)은 슬림화하지 않는다 — 빈번한 호출 시 cross-link 비용이 더 크기 때문.

현재 슬림화된 스킬: **ros1**, **ros2-web-integration** (둘 다 v1.0 미사용).

---

## 출처 추적 (Provenance)

| 스킬 그룹 | 출처 | 라이선스 |
|-----------|------|----------|
| 프로젝트 전용 8종 | 본 프로젝트 (`vmak0314@naver.com` 작성) | 프로젝트 라이선스 |
| 공통 패턴 7종 | 본 프로젝트 (동일) | 동일 |
| 일반 robotics 10종 | [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills) | 해당 저장소 라이선스 확인 필요 |

10종의 일반 robotics 스킬은 [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills) 저장소에서 가져왔다. 본 프로젝트에서 슬림화된 스킬(`ros1`, `ros2-web-integration`)의 `REFERENCE.md`는 그 시점의 원본을 그대로 보존한 것이다. 업스트림 갱신 사항을 반영하려면 위 저장소의 최신 커밋을 확인하고 본 프로젝트의 변경(슬림화·cross-link)을 다시 적용한다.

---

## 신규 스킬 추가 시

1. `.claude/skills/<new-name>/SKILL.md` 생성
2. frontmatter `name`(디렉토리명과 일치) + `description`(트리거 키워드 포함) 작성
3. 이 README의 카테고리 표에 한 줄 추가
4. 본문 마지막 §참고에 외부 출처 명시

---

## 신규 스킬 슬림 시

1. `mv SKILL.md REFERENCE.md`
2. 새 `SKILL.md` 작성 (위 슬림화 정책 §3)
3. 이 README의 표에서 ✂️ 표시 + 라인 수 갱신
