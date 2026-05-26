---
name: demo-collection
description: >
  로봇 demonstration 데이터 수집 가이드 — kinesthetic teaching, teach pendant, scripted demo,
  9종 공구 × 50 demo 절차, 변형 케이스 설계, 데이터 품질 검증, RLDS 변환.
  Track C VLA 학습용 demonstration 수집 시 활성화.
when_to_use: >
  Phase 6a demonstration 수집 환경 구성, demo 녹화 절차 작성,
  변형 케이스 설계, 수집 데이터 품질 검증, RLDS 변환, 라벨링 시.
---

# Demonstration 수집 가이드

> Track C VLA fine-tuning을 위한 데이터 수집. ADR-004 참조. 룰: [`.claude/rules/safety.md`](../rules/safety.md).

## 1. 분량 계획

```
9종 공구 × (fetch + return) × 50 demos/조합 = 900 demonstrations
순수 녹화: 900 × 30초 = 7.5 시간
실제 (실패/재촬영): 20+ 시간
```

### 분배 (다양성 ↑)
| 공구당 50 demo 구성 | 비율 |
|---------------------|------|
| 표준 위치 + 표준 조명 | 25 (50%) |
| 위치 변형 (±2cm) | 10 (20%) |
| 자세 변형 (±10°) | 5 (10%) |
| 조명 변형 (어두움/형광등) | 5 (10%) |
| 거리 변형 (0.5m / 1.2m) | 3 (6%) |
| 의도된 실패 케이스 | 2 (4%) |

## 2. 수집 방법 비교

| 방법 | 정밀도 | 속도 | 학습 적합도 | 비고 |
|------|--------|------|-------------|------|
| **Kinesthetic teaching** (직접 손으로 안내) | 높음 | 빠름 | ★★★ | Doosan e0509는 협동 로봇 — Free Drive 모드 지원 |
| **Teach pendant** (포즈 단위 기록) | 높음 | 느림 | ★★ | 매끄럽지 않음, waypoint 보간 필요 |
| **Scripted demo** (DSR로 자동 실행) | 매우 높음 | 매우 빠름 | ★★ | 다양성 ↓, "perfect" demo 위주 |
| **Tele-operation** (joystick/VR) | 보통 | 보통 | ★★★ | 별도 장비 필요 |

권장: **Kinesthetic teaching** (Doosan Free Drive 모드).

### Doosan Free Drive 모드
```bash
# 티치 펜던트에서:
# Mode → Manual → Direct Teaching → Free Drive 활성화
# 사용자가 로봇을 손으로 자유롭게 움직임
# Python SDK에서도 활성화 가능:
arm.set_robot_mode("manual")
arm.set_compliance_mode(stiffness=[100, 100, 100, 50, 50, 50])
```

## 3. 데이터 형식

### Episode 단위 — RLDS 호환
```python
@dataclass
class Step:
    image: np.ndarray              # (720, 1280, 3) uint8 BGR — D455f RGB
    depth: np.ndarray              # (720, 1280) float32 m — aligned depth (선택)
    state: np.ndarray              # (7,) joints(6) + gripper(1) — rad + normalized
    action: np.ndarray             # (7,) joints_delta(6) + gripper(1) — rad + 0~1
    timestamp: float               # ROS2 Time (절대 시각)

@dataclass
class Episode:
    tool_id: str                   # 'screwdriver_phillips_small'
    intent: str                    # 'fetch' | 'return'
    language: str                  # '필립스 작은 드라이버 가져와' (한국어 원문)
    language_en: str               # 'fetch the small phillips screwdriver' (영어 — VLA 학습용)
    steps: list[Step]              # 일반적으로 200~500 steps @ 10Hz
    success: bool                  # True면 학습 사용, False면 회피
    variant: str                   # 'standard' | 'position_offset' | 'lighting_dark' 등
    operator: str                  # 수집자 이름
    timestamp_start: float
```

### 디렉토리 구조
```
data/
├── demos/
│   ├── screwdriver_phillips_small/
│   │   ├── fetch_001.npz
│   │   ├── fetch_002.npz
│   │   ├── ...
│   │   ├── return_001.npz
│   │   └── ...
│   ├── wrench_8mm/
│   └── ...
├── metadata.parquet              # 모든 demo의 인덱스 (tool_id, intent, variant, 등)
└── rlds/                         # 최종 변환된 RLDS 데이터셋
```

## 4. 수집 도구 (예시 스크립트)

### `collect_demo.py`
```python
import pyrealsense2 as rs
import numpy as np
from pathlib import Path
import time

def collect_episode(
    tool_id: str,
    intent: str,
    language_ko: str,
    variant: str,
    output_dir: Path,
    fps: int = 10,
    max_duration_s: float = 60,
):
    """
    Free Drive 모드에서 사용자가 로봇을 움직이는 동안
    fps Hz로 RGB+state+action 기록.
    """
    arm = DooSanArm(...)
    arm.set_robot_mode("manual")
    arm.enable_free_drive()

    camera = setup_realsense(fps=fps)

    # 카운트다운
    print(f"\n[demo] {tool_id} / {intent} / {variant}")
    print(f"  명령어: {language_ko}")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    print("  녹화 시작 — Enter로 중단")

    steps: list[Step] = []
    prev_state = arm.get_current_posj()
    t_start = time.time()

    try:
        while True:
            if time.time() - t_start > max_duration_s:
                print("  시간 초과")
                break

            # 카메라
            frames = camera.wait_for_frames()
            aligned = rs.align(rs.stream.color).process(frames)
            rgb = np.asanyarray(aligned.get_color_frame().get_data())
            depth = np.asanyarray(aligned.get_depth_frame().get_data())

            # 로봇 상태
            state = np.concatenate([
                arm.get_current_posj(),           # (6,) joints
                [arm.get_gripper_position()],     # (1,) gripper
            ])
            # Action = 다음 step state - 현재 (사후 계산)
            action_placeholder = np.zeros(7)

            steps.append(Step(
                image=rgb, depth=depth, state=state,
                action=action_placeholder,
                timestamp=time.time(),
            ))

            time.sleep(1.0 / fps)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disable_free_drive()
        camera.stop()

    # Action 사후 계산 (state delta)
    for i in range(len(steps) - 1):
        steps[i].action = steps[i+1].state - steps[i].state
    steps = steps[:-1]   # 마지막 step은 action 없음 → 제거

    # 성공/실패 라벨 (사용자 입력)
    success = input("  성공? [y/n]: ").strip().lower() == 'y'

    episode = Episode(
        tool_id=tool_id, intent=intent,
        language=language_ko,
        language_en=translate_to_en(language_ko),
        steps=steps, success=success, variant=variant,
        operator=os.environ.get("USER", "unknown"),
        timestamp_start=t_start,
    )
    save_episode(episode, output_dir)
    print(f"  저장 완료: {len(steps)} steps, {'성공' if success else '실패'}")
```

### Batch 수집 UI (간단)
```python
TOOLS = ["screwdriver_phillips_small", "wrench_8mm", ...]   # 9종
VARIANTS = [
    ("standard", "필립스 작은 드라이버 가져와", "standard"),
    ("position_offset_x", "필립스 작은 드라이버 가져와", "position_offset"),
    ("dim_light", "필립스 작은 드라이버 가져와", "lighting_dark"),
    # ...
]

for tool_id in TOOLS:
    for intent in ["fetch", "return"]:
        for variant_name, lang, variant_type in VARIANTS:
            for i in range(50):
                collect_episode(tool_id, intent, lang, variant_type, output_dir)
```

## 5. 변형 케이스 설계

### 위치 변형
- 슬롯 표준 좌표 ±2cm (x, y 방향 각각)
- 한 demo 당 모두 동일한 변형 — 학습이 변형에 robust해짐

### 조명 변형
- **주광**: 창가, 자연광
- **형광등**: 일반 실내
- **어두움**: 일부 조명 끄기 (D455f IR 영향 확인)

### 자세 변형
- 공구 회전 ±10° (Z축 기준)
- 약간 기울기 (Y축 ±5°)

### 실패 케이스 (의도된)
- 파지 실패 → 재시도
- Staging Area 빈자리 없음 → 거부
- 공구가 슬롯에 없음 (이미 대출 중)

> 실패 케이스도 학습에 포함하면 VLA가 실패 패턴을 인식 → 회복 동작 학습.

## 6. 데이터 품질 검증

### 자동 검사 (수집 직후)
```python
def validate_episode(ep: Episode) -> list[str]:
    """검증 실패 항목 반환 (빈 리스트면 통과)"""
    issues = []

    # 길이
    if len(ep.steps) < 50:
        issues.append(f"너무 짧음 ({len(ep.steps)} steps)")
    if len(ep.steps) > 1000:
        issues.append(f"너무 김 ({len(ep.steps)} steps)")

    # Joint 범위
    for s in ep.steps:
        if np.any(np.abs(s.state[:6]) > 3.14):
            issues.append("joint 값이 ±π 초과")
            break

    # Action 크기
    max_action = max(np.linalg.norm(s.action[:6]) for s in ep.steps)
    if max_action > 0.5:
        issues.append(f"action delta 큼 ({max_action:.2f}) — 갑작스러운 변화")

    # 이미지 품질
    for s in ep.steps[:3]:    # 처음 3개만 샘플 검사
        if s.image.mean() < 20:
            issues.append("이미지 너무 어두움")
        if s.image.std() < 5:
            issues.append("이미지 단색 (카메라 문제?)")

    return issues

# 사용
for ep_path in data_dir.glob("**/*.npz"):
    ep = load_episode(ep_path)
    issues = validate_episode(ep)
    if issues:
        print(f"{ep_path}: {', '.join(issues)}")
```

### 시각적 검수 (10% 샘플링)
```python
# 무작위 90 demo (10%) 영상 재생 + 사람 확인
import random
samples = random.sample(all_demos, k=int(len(all_demos) * 0.1))
for ep in samples:
    render_episode_video(ep, output=f"review/{ep.tool_id}_{ep.intent}_{ep.id}.mp4")
# 사람이 보고 명백한 오류는 제외 표시
```

## 7. 데이터 라벨링

### 자동 라벨 (수집 시 기록)
- tool_id, intent, variant, success: 수집자가 입력
- language_ko: 사전 정의된 명령어 풀에서 선택
- language_en: 번역 자동 또는 수동

### 명령어 다양성 (학습 일반화 ↑)
```python
COMMAND_VARIATIONS = {
    "screwdriver_phillips_small": [
        "필립스 작은 드라이버 가져와",
        "작은 필립스 줘",
        "small phillips 가져와줘",
        "십자 작은 거 줘",
    ],
    # ...
}
```
한 공구당 4~5가지 표현 → 자연어 다양성 학습

## 8. RLDS 변환

```python
# convert_to_rlds.py
import tensorflow_datasets as tfds
import rlds

class RobotArmDataset(tfds.core.GeneratorBasedBuilder):
    VERSION = tfds.core.Version("1.0.0")

    def _info(self):
        return tfds.core.DatasetInfo(
            builder=self,
            features=tfds.features.FeaturesDict({
                "steps": tfds.features.Dataset({
                    "observation": tfds.features.FeaturesDict({
                        "image": tfds.features.Image(shape=(720, 1280, 3)),
                        "state": tfds.features.Tensor(shape=(7,), dtype=tf.float32),
                    }),
                    "action": tfds.features.Tensor(shape=(7,), dtype=tf.float32),
                    "language_instruction": tfds.features.Text(),
                    "is_first": tf.bool,
                    "is_last": tf.bool,
                    "is_terminal": tf.bool,
                }),
            }),
        )

    def _generate_examples(self, demo_dir):
        for ep_path in Path(demo_dir).glob("**/*.npz"):
            ep = load_episode(ep_path)
            if not ep.success:
                continue   # 실패 demo 제외 또는 별도 처리
            yield str(ep_path), {
                "steps": [{
                    "observation": {"image": s.image, "state": s.state.astype(np.float32)},
                    "action": s.action.astype(np.float32),
                    "language_instruction": ep.language_en,
                    "is_first": i == 0,
                    "is_last": i == len(ep.steps) - 1,
                    "is_terminal": i == len(ep.steps) - 1,
                } for i, s in enumerate(ep.steps)],
            }

# 빌드
tfds build robot_arm_dataset
# 결과: ~/tensorflow_datasets/robot_arm_dataset/1.0.0/
```

## 9. 데이터 보관

| 항목 | 권장 |
|------|------|
| 저장소 | NAS 또는 S3 (대용량 대비) |
| 백업 | 별도 디스크 + 클라우드 (이중화) |
| 압축 | RGB는 JPEG 또는 H.264 비디오 / Depth는 PNG (uint16 보존) |
| Git 포함? | ❌ 절대 금지 (LFS도 비추천 — 데이터셋이 큼) |

## 10. 흔한 함정

### ❌ 다양성 부족
- 표준 위치만 50회 → 변형에 generalize 실패
- ✅ 변형 케이스 비율 명확히 (위 §1 참조)

### ❌ Action 잘못 계산
- state delta 대신 absolute joint를 action으로 저장
- ✅ 데이터 스펙에 따라 (OpenVLA는 delta, Octo는 absolute) 명확히

### ❌ 카메라 frame 일관성 깨짐
- 수집 중 카메라 위치 변경
- ✅ hand-eye 캘리브 1회 + 수집 동안 고정

### ❌ 시간 단위 불일치
- 일부 demo는 5Hz, 일부는 30Hz
- ✅ 전체 통일 (10Hz 권장 — 0.1초 간격)

### ❌ 성공/실패 라벨 부정확
- "거의 성공"을 success로 분류 → 학습 노이즈
- ✅ 명확한 기준 + 의심스러우면 fail로 분류 후 별도 처리

### ❌ 안전 무시
- Free Drive 모드에서 빠른 동작 → 충돌 위험
- ✅ Free Drive 중 속도 제한 + 명확한 작업 영역

### ❌ 한국어/영어 명령 불일치
- 한국어로 수집했는데 VLA는 영어 학습됨
- ✅ language_en 필드를 일관되게 + 번역 품질 검수

## 11. 룰 매핑

| 룰 | 적용 |
|----|------|
| S-5 | Free Drive 중에도 joint/속도 한계 준수 |
| E-1 | rad / m / ROS2 Time 통일 |
| E-4 | 수집 파라미터 (fps, 분량 등)는 `config/demo_collection.yaml` |
| P-3 | 데이터 경로 등 환경 변수 → `.env` |

## 12. 참고

- Open-X-Embodiment 수집 가이드: <https://robotics-transformer-x.github.io/>
- RLDS: <https://github.com/google-research/rlds>
- LeRobot data format: <https://github.com/huggingface/lerobot>
- Doosan Direct Teaching 매뉴얼: 공식 문서 참조
- 관련 스킬: [`vla-finetuning`](vla-finetuning.md), [`doosan-e0509`](doosan-e0509.md), [`realsense-d455f`](realsense-d455f.md)
- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md), [`.claude/rules/engineering.md`](../rules/engineering.md), [`.claude/rules/process.md`](../rules/process.md)
