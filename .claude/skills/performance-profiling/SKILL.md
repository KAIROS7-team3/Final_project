---
name: performance-profiling
description: >
  Python 성능 프로파일링 — cProfile, line_profiler, memory_profiler,
  VRAM 모니터링, async 병목, 로봇 시스템 사이클 타임 측정.
  성능 최적화, 병목 탐지, 사이클 타임 ≤13초 달성 시 활성화.
when_to_use: >
  사이클 타임 초과, VLA 추론 지연, 메모리 누수, VRAM 부족,
  CPU 병목 탐지, async 루프 지연, 프로파일링 환경 설정 시.
---

# Python 성능 프로파일링

> 이 프로젝트 목표: 사이클 타임 ≤ 13초 (음성 → Staging Area 거치 완료). VLA 추론 ≤ 2.5초.

## 1. 사이클 타임 측정 — 먼저 측정, 그 다음 최적화

```python
import time
from contextlib import contextmanager
from dataclasses import dataclass, field

@dataclass
class CycleProfiling:
    segments: dict[str, float] = field(default_factory=dict)
    _start: float = field(default=0.0, init=False)

    @contextmanager
    def segment(self, name: str):
        t = time.perf_counter()
        try:
            yield
        finally:
            self.segments[name] = time.perf_counter() - t

    def report(self) -> str:
        total = sum(self.segments.values())
        lines = [f"총 사이클: {total*1000:.0f}ms"]
        for name, elapsed in self.segments.items():
            pct = elapsed / total * 100
            lines.append(f"  {name:<25} {elapsed*1000:6.0f}ms ({pct:.0f}%)")
        return "\n".join(lines)

# 사용
prof = CycleProfiling()
with prof.segment("stt"):
    text = whisper.transcribe(audio)
with prof.segment("intent_parse"):
    intent = parse_intent(text)
with prof.segment("db_gate"):
    check_feasibility(intent.tool_id)
with prof.segment("vla_infer"):
    trajectory = vla.predict(obs, instruction=text)
with prof.segment("safety_check"):
    safety_validator.check(trajectory)
with prof.segment("arm_execute"):
    arm.execute(trajectory)

print(prof.report())
# 총 사이클: 9800ms
#   stt                        800ms (8%)
#   vla_infer                 4200ms (43%)   ← 병목
#   arm_execute               4000ms (41%)
```

## 2. cProfile — 함수 레벨 프로파일링

```python
import cProfile
import pstats
import io

# 코드 블록 프로파일링
pr = cProfile.Profile()
pr.enable()

vla.predict(obs, instruction=text)   # 대상 코드

pr.disable()

# 결과 출력
stream = io.StringIO()
ps = pstats.Stats(pr, stream=stream).sort_stats("cumulative")
ps.print_stats(20)   # 상위 20개 함수
print(stream.getvalue())
```

```bash
# CLI
python -m cProfile -s cumulative track_c_vla.py > profile.txt

# snakeviz로 시각화
pip install snakeviz
python -m cProfile -o output.prof track_c_vla.py
snakeviz output.prof
```

## 3. line_profiler — 라인 레벨 프로파일링

```bash
pip install line_profiler
```

```python
# 특정 함수 라인별 시간 측정
@profile   # line_profiler 데코레이터
def preprocess_observation(rgb: np.ndarray, state: np.ndarray):
    img = cv2.resize(rgb, (224, 224))          # 이 줄이 얼마나?
    img = img.astype(np.float32) / 255.0
    img = torch.from_numpy(img).permute(2, 0, 1)
    img = img.unsqueeze(0).to("cuda")
    return img, torch.tensor(state, device="cuda")
```

```bash
kernprof -l -v track_c_vla.py
# Line #   Hits   Time  Per Hit   % Time  Line Contents
# 10          1  12000  12000.0     82.1  img = cv2.resize(...)   ← 병목
```

## 4. memory_profiler — 메모리 사용량

```bash
pip install memory_profiler
```

```python
from memory_profiler import profile

@profile
def load_vla_model(checkpoint_path: str):
    model = VLAModel.from_pretrained(checkpoint_path)   # 여기서 피크?
    model.eval()
    return model
```

```bash
python -m memory_profiler load_model.py
# Line #    Mem usage    Increment   Line Contents
# 5       1200.0 MiB    1200.0 MiB  model = VLAModel.from_pretrained(...)
```

### 메모리 누수 탐지 (tracemalloc)
```python
import tracemalloc

tracemalloc.start()

for _ in range(100):
    obs = camera.get_observation()    # 루프마다 누수?

snapshot = tracemalloc.take_snapshot()
top_stats = snapshot.statistics("lineno")
for stat in top_stats[:5]:
    print(stat)
# perception.py:45: 500.0 KiB (x100 = 50MB/cycle) ← 누수
```

## 5. VRAM 모니터링 (GPU)

```python
import torch

def vram_used_gb() -> float:
    if not torch.cuda.is_available():
        return 0.0
    return torch.cuda.memory_allocated() / 1024**3

def vram_peak_gb() -> float:
    return torch.cuda.max_memory_allocated() / 1024**3

# 추론 전후 비교
torch.cuda.reset_peak_memory_stats()
vram_before = vram_used_gb()

output = vla.predict(obs, instruction=text)

print(f"VRAM 사용: {vram_used_gb():.2f}GB (피크: {vram_peak_gb():.2f}GB)")
# Vector 16 HX (16GB VRAM) 기준: 피크 < 14GB 목표
```

```bash
# 터미널에서 실시간 모니터링
watch -n 1 nvidia-smi
nvidia-smi dmon -s mu   # 1초 간격 메모리/활용률
```

### 추론 속도 벤치마크
```python
import time

N = 10
torch.cuda.synchronize()
t0 = time.perf_counter()
for _ in range(N):
    out = vla.predict(obs, instruction=text)
    torch.cuda.synchronize()   # GPU 완료 대기
elapsed_ms = (time.perf_counter() - t0) / N * 1000
print(f"VLA 추론 평균: {elapsed_ms:.0f}ms (목표: ≤2500ms)")
```

## 6. Async 병목 — asyncio

```python
import asyncio
import time

# 느린 코루틴 탐지
async def slow_fn():
    loop = asyncio.get_event_loop()
    slow_threshold = 0.1   # 100ms 이상 = 느림

    tasks = asyncio.all_tasks(loop)
    for task in tasks:
        ...

# 실제 측정
async def timed(coro, label: str):
    t0 = time.perf_counter()
    result = await coro
    elapsed = time.perf_counter() - t0
    if elapsed > 0.1:
        logger.warning("느린 코루틴 %s: %.0fms", label, elapsed * 1000)
    return result
```

```python
# asyncio debug 모드 (이벤트 루프 블로킹 감지)
import asyncio
asyncio.run(main(), debug=True)
# asyncio: Executing <Task ...> took 0.215 seconds
```

## 7. 공통 병목 패턴 + 해결

### NumPy → GPU 복사 반복
```python
# ❌ 루프마다 CPU→GPU 복사
for step in trajectory:
    tensor = torch.tensor(step).cuda()   # 느림

# ✅ 한 번에 복사
tensor_batch = torch.tensor(trajectory, device="cuda")
```

### 불필요한 np.copy
```python
# ❌
obs_copy = np.copy(obs)   # 필요한가?

# ✅ 읽기만 한다면 그대로 사용
```

### 반복 모델 로드
```python
# ❌ 매 요청마다 로드
def infer(obs):
    model = VLAModel.from_pretrained(...)   # 수십 초

# ✅ 모듈 레벨 싱글톤
_model: VLAModel | None = None
def get_model() -> VLAModel:
    global _model
    if _model is None:
        _model = VLAModel.from_pretrained(CHECKPOINT)
    return _model
```

### 동기 I/O in async
```python
# ❌ async 함수 안에서 blocking I/O
async def fetch_config():
    data = open("config/vla.yaml").read()   # 이벤트 루프 블록

# ✅ run_in_executor 또는 aiofiles
import aiofiles
async def fetch_config():
    async with aiofiles.open("config/vla.yaml") as f:
        return await f.read()
```

## 8. 프로파일링 워크플로

```
1. 측정 → CycleProfiling으로 세그먼트별 시간 확인
2. 병목 식별 → 전체의 20% 이상 차지하는 세그먼트
3. 세부 측정 → 병목 함수에 line_profiler 적용
4. 최적화 → 한 번에 한 가지만 변경
5. 재측정 → 개선됐는지 수치로 확인
6. 반복
```

> 프로파일 없이 최적화하지 말 것. 직관은 대부분 틀린다.

## 9. 참고

- cProfile: <https://docs.python.org/3/library/profile.html>
- line_profiler: <https://github.com/pyutils/line_profiler>
- memory_profiler: <https://github.com/pythonprofilers/memory_profiler>
- snakeviz: <https://jiffyclub.github.io/snakeviz/>
- 관련 스킬: [`vla-finetuning`](vla-finetuning.md) §8 (추론 속도 측정), [`python-patterns`](python-patterns.md)
