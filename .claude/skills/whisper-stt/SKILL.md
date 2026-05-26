---
name: whisper-stt
description: >
  OpenAI Whisper STT 통합 가이드 — 모델 선택, 한국어 처리, VAD 게이팅,
  로봇 작업 중 오인식 방지, ROS2 voice 패키지 통합, 실시간 처리.
  Track A/B voice 패키지, Track C 직접 호출 구현 시 활성화.
when_to_use: >
  Whisper 모델 선택, 한국어 STT 구현, VAD(음성 활성 감지) 적용,
  ROS2 voice 노드 작성, is_moving 연동 오인식 방지,
  음성 명령 전처리 파이프라인 설계 시.
---

# Whisper STT 통합 가이드

> Track A/B: `voice/` 패키지. Track C: `track_c_vla.py`에서 직접 호출.
> 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-1 (단위 일관성 — 오디오 샘플레이트 명시).

## 1. 모델 선택

| 모델 | 파라미터 | VRAM | 한국어 WER | 실시간 속도 |
|------|---------|------|-----------|-----------|
| `tiny` | 39M | ~0.5GB | 낮음 | ~10× | 한국어 부정확 |
| `base` | 74M | ~1GB | 보통 | ~7× | |
| **`small`** | 244M | ~2GB | **양호** | ~4× | **이 프로젝트 권장** |
| `medium` | 769M | ~5GB | 우수 | ~2× | VRAM 여유 있을 때 |
| `large-v3` | 1550M | ~10GB | 최고 | ~1× | Track C VRAM 여유 없음 |

> Vector 16 HX (16GB VRAM): VLA 모델이 대부분의 VRAM을 사용하므로 Track C에서는 `small` 필수.

## 2. 설치 및 기본 사용

```bash
pip install openai-whisper
# ffmpeg 필요
sudo apt install ffmpeg
```

```python
import whisper
import numpy as np

# 모델 로드 (최초 1회 — 싱글톤 패턴 권장)
model = whisper.load_model("small", device="cuda")

def transcribe(audio_np: np.ndarray, sample_rate: int = 16000) -> str:
    """
    Args:
        audio_np: float32 [-1, 1], shape (N,)
        sample_rate: 샘플레이트 (Whisper 내부 16kHz로 리샘플링)
    """
    if sample_rate != 16000:
        audio_np = whisper.audio.resample(audio_np, sample_rate, 16000)

    result = model.transcribe(
        audio_np,
        language="ko",           # 한국어 고정 (자동 감지 비활성화)
        task="transcribe",
        fp16=True,               # VRAM 절약
        temperature=0.0,         # 결정론적 출력
        condition_on_previous_text=False,  # 이전 문맥 참조 끄기 (단발 명령)
    )
    return result["text"].strip()
```

## 3. VAD — 음성 활성 구간만 처리

Whisper는 무음 구간에서도 환각(hallucination)을 출력한다. VAD로 필터링 필수.

```bash
pip install silero-vad
```

```python
import torch

# Silero VAD 모델 (로컬 캐시)
vad_model, utils = torch.hub.load(
    repo_or_dir="snakers4/silero-vad",
    model="silero_vad",
    trust_repo=True,
)
(get_speech_timestamps, _, read_audio, *_) = utils

def has_speech(audio_np: np.ndarray, sample_rate: int = 16000,
               threshold: float = 0.5) -> bool:
    """음성 포함 여부 반환."""
    audio_tensor = torch.tensor(audio_np)
    timestamps = get_speech_timestamps(
        audio_tensor, vad_model,
        threshold=threshold,
        sampling_rate=sample_rate,
    )
    return len(timestamps) > 0

# 파이프라인
def stt_pipeline(audio_np: np.ndarray) -> str | None:
    if not has_speech(audio_np):
        return None   # VAD 통과 안 함 → 무시
    return transcribe(audio_np)
```

## 4. is_moving 연동 — 로봇 작동 중 오인식 방지

로봇 모터 소음이 음성으로 오인식되는 것을 방지.

```python
# Track A/B: ROS2 토픽 구독
class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_node")
        self._is_moving = False

        # 로봇 상태 구독
        self.create_subscription(
            Bool, "/dsr/is_moving", self._on_moving_status, 10
        )

    def _on_moving_status(self, msg: Bool) -> None:
        self._is_moving = msg.data

    def _on_audio_chunk(self, audio_np: np.ndarray) -> None:
        if self._is_moving:
            return   # 로봇 작동 중 → 음성 무시

        text = stt_pipeline(audio_np)
        if text:
            self._publish_command(text)
```

```python
# Track C: 상태 플래그
class TrackCPipeline:
    def __init__(self):
        self._is_moving = False

    def set_moving(self, moving: bool) -> None:
        self._is_moving = moving

    def listen_loop(self) -> None:
        while True:
            audio = mic.record(duration_s=2.0)
            if self._is_moving:
                continue
            text = stt_pipeline(audio)
            if text:
                self.handle_command(text)
```

## 5. 실시간 마이크 입력 (pyaudio)

```bash
pip install pyaudio
```

```python
import pyaudio
import numpy as np
from collections import deque
import threading

SAMPLE_RATE = 16000
CHUNK_SIZE = 1024          # 64ms per chunk
RECORD_SECONDS = 3.0       # 한 번에 녹음할 최대 길이
SILENCE_THRESHOLD = 0.02   # RMS 기반 무음 감지

class MicRecorder:
    def __init__(self):
        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

    def record_utterance(self, max_duration_s: float = 4.0) -> np.ndarray:
        """발화 감지 후 무음 때 종료."""
        chunks = []
        silence_count = 0
        speaking = False

        for _ in range(int(SAMPLE_RATE / CHUNK_SIZE * max_duration_s)):
            chunk = np.frombuffer(
                self._stream.read(CHUNK_SIZE, exception_on_overflow=False),
                dtype=np.float32,
            )
            rms = float(np.sqrt(np.mean(chunk ** 2)))

            if rms > SILENCE_THRESHOLD:
                speaking = True
                silence_count = 0
                chunks.append(chunk)
            elif speaking:
                chunks.append(chunk)
                silence_count += 1
                if silence_count > 15:   # 약 1초 무음 → 종료
                    break

        if not chunks:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(chunks)

    def close(self):
        self._stream.stop_stream()
        self._stream.close()
        self._pa.terminate()
```

## 6. ROS2 통합 (Track A/B voice 패키지)

```python
# voice/whisper_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class WhisperNode(Node):
    def __init__(self):
        super().__init__("whisper_node")

        # 파라미터 (config/voice_node.yaml에서)
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("vad_threshold", 0.5)
        self.declare_parameter("sample_rate_hz", 16000)

        model_name = self.get_parameter("whisper_model").value
        self._model = whisper.load_model(model_name, device="cuda")
        self._recorder = MicRecorder()

        self._pub = self.create_publisher(String, "/voice/command", 10)
        self.create_subscription(Bool, "/dsr/is_moving", self._on_moving, 10)
        self._is_moving = False

        self.create_timer(0.1, self._poll)

    def _on_moving(self, msg: Bool):
        self._is_moving = msg.data

    def _poll(self):
        if self._is_moving:
            return
        audio = self._recorder.record_utterance()
        if len(audio) == 0:
            return
        text = stt_pipeline(audio)
        if text:
            self.get_logger().info("인식: %s", text)
            msg = String()
            msg.data = text
            self._pub.publish(msg)
```

```yaml
# config/voice_node.yaml
voice_node:
  ros__parameters:
    whisper_model: small
    vad_threshold: 0.5
    sample_rate_hz: 16000
```

## 7. 한국어 처리 팁

### 공구 이름 오인식 보정
```python
# 음성 인식 결과를 공구 사전으로 매핑
TOOL_ALIASES = {
    "드라이버": "screwdriver",
    "필립스": "phillips",
    "플러스": "phillips",      # 오인식 패턴
    "마이너스": "flathead",
    "렌치": "wrench",
    "플라이어": "pliers",
}

def normalize_tool_name(text: str) -> str:
    for alias, canonical in TOOL_ALIASES.items():
        text = text.replace(alias, canonical)
    return text
```

### 초기 프롬프트로 정확도 향상
```python
result = model.transcribe(
    audio_np,
    language="ko",
    initial_prompt="공구함, 드라이버, 렌치, 플라이어, 가져와, 반납",
    # 도메인 단어를 힌트로 제공
)
```

## 8. 흔한 함정

### ❌ 모델을 매번 로드
```python
def transcribe(audio):
    model = whisper.load_model("small")   # 매번 수 초 소요
```
✅ 싱글톤 또는 클래스 속성으로 캐싱

### ❌ VAD 없이 Whisper 직접 적용
- 무음 → "음..." "감사합니다" 등 환각 출력
- ✅ VAD로 음성 구간 확인 후 Whisper 호출

### ❌ is_moving 확인 없이 연속 인식
- 모터 소음이 명령으로 인식 → 의도치 않은 동작
- ✅ `/dsr/is_moving` 또는 플래그로 차단

### ❌ GPU OOM (VLA와 VRAM 공유)
- Whisper large + VLA Q4 → 16GB 초과 가능
- ✅ Whisper small (2GB) + VLA Q4 (4–5GB) = 안전

### ❌ float16 오디오 입력
- Whisper는 float32 기대
- ✅ `audio.astype(np.float32)` 확인

## 9. 참고

- OpenAI Whisper: <https://github.com/openai/whisper>
- Silero VAD: <https://github.com/snakers4/silero-vad>
- 관련 스킬: [`vla-finetuning`](vla-finetuning.md), [`realsense-d455f`](realsense-d455f.md)
- 설정: `config/voice_node.yaml` (ROS2 파라미터)
