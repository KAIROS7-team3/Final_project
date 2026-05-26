---
name: vla-finetuning
description: >
  Vision-Language-Action 모델 파인튜닝 가이드 — OpenVLA·π0·Octo 비교,
  VRAM 예산·Q4 양자화, demonstration 형식(RLDS), fine-tuning 환경, 평가 메트릭.
  Track C VLA 모델 선정 및 파인튜닝 시 활성화.
when_to_use: >
  VLA 모델 선정 (미결 #5), OpenVLA/π0 파인튜닝, Q4 양자화, LoRA 적용,
  클라우드 GPU 환경 셋업, Track C 추론 성능 평가 시.
  (demonstration 수집 절차·형식은 demo-collection 스킬 전담)
---

# VLA 모델 파인튜닝 가이드

> Track C 전용. 룰: [`.claude/rules/safety.md`](../rules/safety.md) S-1 (SafetyValidator 우회 금지), [`.claude/rules/engineering.md`](../rules/engineering.md).
> 관련 미결 사항: #5 (VLA 모델 선정, Phase 6 전), #22 (입력 형식).

## 1. 후보 모델 비교

| 모델 | 파라미터 | 베이스 | 입력 | 출력 | 라이선스 | 비고 |
|------|---------|--------|------|------|---------|------|
| **OpenVLA-7B** | 7B | Llama-2 + DINOv2 + SigLIP | RGB + text | discretized actions (256 bins) | MIT | 가장 널리 검증된 오픈소스. Open-X-Embodiment 학습 |
| **π0** (FAST) | ~3B | PaliGemma + flow matching | RGB + text | continuous action (chunk 50 steps) | Apache 2.0 | Physical Intelligence. action chunking으로 부드러움 |
| **Octo** | 27M / 93M | Transformer scratch | RGB + RGB-wrist + text | continuous diffusion | MIT | 작고 빠름. 적은 demonstration으로 학습 가능 |
| **RT-2** | 미공개 | PaLM-E | RGB + text | discretized actions | (Google 비공개) | 학술 참고용 |
| **RDT-1B** | 1B | DiT | RGB + RGB-wrist + text | continuous diffusion | MIT | Bimanual에 강함, 중국어 시연 우수 |

### Vector 16 HX (16GB VRAM) 적합도
| 모델 | BF16 추론 | Q4 추론 | Fine-tune (LoRA) | Fine-tune (Full) |
|------|----------|---------|-----------------|------------------|
| OpenVLA-7B | ❌ 14GB+ 위험 | ✅ 4-5GB | ⚠️ 보통 클라우드 권장 | ❌ 불가 |
| π0 | ⚠️ ~10GB | ✅ 3-4GB | ✅ 가능 | ❌ 불가 (24GB+) |
| Octo (93M) | ✅ ~1GB | ✅ <0.5GB | ✅ 매우 가능 | ✅ 가능 |
| RDT-1B | ⚠️ ~4GB | ✅ 2GB | ✅ 가능 | ⚠️ 빠듯 |

> Vector 16 HX 단독으로는 **Octo 또는 π0 (Q4)** 권장. OpenVLA fine-tune은 클라우드 필요.

## 2. 권장 선택 기준 (이 프로젝트)

| 우선순위 | 권장 |
|----------|------|
| **빠른 PoC** | Octo 93M — 작고 빠름, 적은 demo로 학습 |
| **품질 우선** | π0 (Q4 추론) — action chunking 우수 |
| **검증된 베이스라인** | OpenVLA-7B (클라우드 fine-tune + 로컬 Q4 추론) |

이 프로젝트는 **9종 공구 × fetch+return**의 좁은 도메인 → Octo로 시작 권장.

## 3. Demonstration 데이터 형식

### RLDS (Reinforcement Learning Datasets) — Open-X-Embodiment 표준
```python
# 1개 episode 구조
{
    "steps": [
        {
            "observation": {
                "image": np.ndarray(H, W, 3, dtype=np.uint8),      # RGB
                "wrist_image": np.ndarray(...),                    # 선택 (이 프로젝트 없음)
                "state": np.ndarray(7,),                           # joint(6) + gripper(1)
            },
            "action": np.ndarray(7,),                              # joint_delta(6) + gripper(1)
            "language_instruction": "phillips 작은 드라이버 가져와",
            "is_first": True,
            "is_last": False,
            "reward": 0.0,
            "discount": 1.0,
        },
        # ... 200~500 steps per episode
    ]
}
```

### 변환 도구
```bash
# RLDS Dataset Builder
pip install rlds tensorflow_datasets

# OpenVLA의 dataset spec 참고
# https://github.com/openvla/openvla/tree/main/vla-scripts/extern
```

### Octo 형식 (간단)
```python
# numpy / pickle
demo = {
    "obs": {
        "image": np.ndarray(T, H, W, 3),     # T = timesteps
        "proprio": np.ndarray(T, 7),
    },
    "action": np.ndarray(T, 7),
    "language_instruction": "phillips small screwdriver fetch",
}
```

## 4. 데이터 수집

분량·변형 케이스·수집 절차·RLDS 변환은 [`demo-collection.md`](demo-collection.md) 전담.
fine-tuning 시 입력으로 사용하는 데이터 형식은 §3에서 다룬다.

## 5. Fine-tuning 환경

### 로컬 옵션 (제한적)
```bash
# Vector 16 HX (RTX 4090 Laptop = 16GB) — Octo 93M 가능
# π0 Q4 LoRA — 가능
# OpenVLA-7B fine-tune — 불가능

# LoRA로 메모리 절약
pip install peft bitsandbytes
```

### 클라우드 옵션 (OpenVLA 등)
| 서비스 | GPU | 비고 |
|--------|-----|------|
| **Lambda Labs** | A100 80GB | 가용성 변동, 실시간 가격 확인 필요 |
| **Vast.ai** | A100 80GB | 저렴하나 신뢰성 변동 |
| **RunPod** | A100 80GB | 안정적, 컨테이너 친화 |
| **Google Colab Pro+** | A100 40GB | 월정액, 사용 한도 있음 |

```bash
# Lambda Labs 예시
ssh ubuntu@<lambda-ip>
git clone https://github.com/openvla/openvla
cd openvla
pip install -e .
# 데이터 업로드 (rsync, s3 등)
# 학습 실행
torchrun --nproc-per-node 8 \
    vla-scripts/finetune.py \
    --vla_path "openvla/openvla-7b" \
    --data_root_dir <data> \
    --dataset_name our_dataset \
    --run_root_dir <output> \
    --use_lora True \
    --lora_rank 32
```

### 시크릿 관리 (룰 P-3)
```bash
# .env (gitignored)
LAMBDA_LABS_API_KEY=ll-xxxxx
HUGGINGFACE_TOKEN=hf_xxxxx
S3_ACCESS_KEY=xxxxx
```

## 6. LoRA — 메모리 절약 + 빠른 학습

Full fine-tuning 대신 LoRA(Low-Rank Adaptation) 권장.

```python
from peft import LoraConfig, get_peft_model

base_model = AutoModel.from_pretrained("openvla/openvla-7b")
lora_config = LoraConfig(
    r=32,                          # rank
    lora_alpha=64,
    target_modules=["q_proj", "v_proj"],  # 모델 구조에 따라
    lora_dropout=0.05,
    bias="none",
    task_type="CAUSAL_LM",
)
model = get_peft_model(base_model, lora_config)
# 학습 가능 파라미터: 전체의 ~0.5% (메모리 1/10)
```

### LoRA + Q4 양자화 (QLoRA)
```python
from transformers import BitsAndBytesConfig
import torch

bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_compute_dtype=torch.bfloat16,
    bnb_4bit_quant_type="nf4",
)
base = AutoModel.from_pretrained("openvla/openvla-7b", quantization_config=bnb_config)
# 16GB VRAM에서 OpenVLA fine-tune 가능
```

## 7. 학습 모니터링

```python
# Weights & Biases
import wandb
wandb.init(project="robot-arm-vla", name="openvla-7b-tools-v1")

# 핵심 지표
# - train/loss
# - train/action_l1                 # action L1 오차
# - eval/success_rate (있다면)
# - eval/action_l1_per_joint        # joint별 정확도
```

### 학습 epoch 권장
- OpenVLA fine-tune: 5~10 epochs (900 demos 기준 ~50k steps)
- Octo: 50~100 epochs (모델 작아서 더 많은 epoch 필요)

## 8. 추론 — Track C 통합

```python
# track_c_vla.py
from vla_model import VLAModel

vla = VLAModel.from_pretrained(
    "checkpoints/openvla-tools-v1",
    quantization="Q4",            # Vector 16 HX에서 추론
    device="cuda:0",
)

def infer_action(rgb: np.ndarray, depth: np.ndarray, text: str,
                 robot_state: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Returns:
        joint_trajectory: (T, 6) rad
        gripper_command: float 0~1 (0=open, 1=close)
    """
    obs = {"image": rgb, "state": robot_state}
    output = vla.predict(obs, instruction=text)
    return output.joint_traj, output.gripper

# 사용
joint_traj, gripper_cmd = infer_action(rgb, depth, text, state)
# 항상 SafetyValidator 통과 (.claude/rules/safety.md S-1)
if safety_validator.check(joint_traj):
    arm.execute(joint_traj)
```

### 추론 속도 측정
```python
import time
t0 = time.perf_counter()
for _ in range(10):
    out = vla.predict(obs, instruction=text)
elapsed = (time.perf_counter() - t0) / 10
print(f"추론 평균: {elapsed*1000:.1f}ms")
# 목표: ≤2.5초 (rules의 사이클 타임 ≤13초 만족)
```

## 9. 평가 메트릭

### 학습 단계
- **Action L1 loss**: joint별 평균 절대 오차 (rad)
- **Validation set**: 학습에 사용 안 한 demo로 평가

### 통합 단계 (Phase 7 비교 평가)
- **Success rate**: 9종 × 3 cycle = 27 trial 중 성공률
- **Action error rate ≤ 3%**: 잘못된 공구 파지 또는 잘못된 위치 거치 (수락 기준)
- **사이클 타임 ≤ 13초**: 음성 → Staging Area 거치 완료 (목표)
- **SafetyValidator rejection rate**: 비정상 trajectory 비율

## 10. 흔한 함정

### ❌ Demonstration 다양성 부족
- 표준 위치만 50회 녹화 → 변형에 generalize 실패
- ✅ 변형 케이스 (위치, 조명, 자세) 포함 + 실패 demo도 일부 포함

### ❌ Action 단위 / frame 불일치
- 학습 시 rad / 추론 시 degree 혼용
- ✅ RLDS 메타데이터에 단위 명시 + 변환 함수 고정

### ❌ State 정규화 안 함
- joint 값 범위 차이 (J1: ±π, J5: ±0.85π)로 학습 불안정
- ✅ z-score 또는 [-1, 1] 정규화

### ❌ Camera frame 불일치
- 학습 데이터는 eye-in-hand, 배포 시 eye-to-hand
- ✅ 학습 시 사용한 카메라 위치 / hand-eye 행렬 그대로 유지

### ❌ Safety bypass
- VLA 출력을 직접 SDK로 보냄 (SafetyValidator 우회)
- ✅ **절대 금지** (`.claude/rules/safety.md` S-1)

### ❌ 양자화 후 정확도 점검 안 함
- Q4로 변환 후 추론 정확도 ~5% 하락 가능
- ✅ BF16 → Q4 변환 후 동일 평가 세트로 재검증

### ❌ Action chunking 길이
- π0의 50-step chunk를 그대로 사용 → 우리 ~5초 작업에 너무 김
- ✅ 데이터에 맞춰 chunk 크기 조정 (10~20 step 권장)

## 11. 룰 매핑

| 룰 | 적용 |
|----|------|
| S-1 | VLA 출력은 무조건 SafetyValidator 통과 후 SDK 호출 |
| S-2 | DB gate(`check_feasibility`)는 VLA 호출 전 실행 |
| E-1 | 학습/추론 모두 rad 단위. action 표현에 명시 |
| E-4 | 모델 경로, threshold 등은 `config/vla.yaml` |
| P-3 | 클라우드 토큰은 `.env`. commit 금지 |

## 12. 참고

- OpenVLA: <https://openvla.github.io/>
- π0 (Physical Intelligence): <https://www.physicalintelligence.company/blog/pi0>
- Octo: <https://octo-models.github.io/>
- Open-X-Embodiment: <https://robotics-transformer-x.github.io/>
- RLDS: <https://github.com/google-research/rlds>
- HuggingFace LeRobot: <https://github.com/huggingface/lerobot>
- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md), [`.claude/rules/engineering.md`](../rules/engineering.md)
- 관련 스킬: [`demo-collection`](demo-collection.md), [`doosan-e0509`](doosan-e0509.md), [`realsense-d455f`](realsense-d455f.md)
