# voice package

Track A/B에서 마이크 입력을 Whisper STT로 변환하고, STT 원문을 프로젝트
표준 `Intent` 메시지로 변환하는 패키지다.

Pipeline:

```text
PyAudio microphone -> whisper_node -> /voice/raw_text
/voice/raw_text -> gemma_intent_node -> DB Gate -> /voice/intent
```

`whisper_node`는 원문 텍스트만 publish한다. `fetch`/`return` 가능 여부와
DB Gate 통과 여부는 `gemma_intent_node`와 `db` 패키지가 담당한다.

## Dependencies

ROS2/ament:

- ROS2 Humble
- `ament_python`
- `rclpy`
- `std_msgs`
- `interfaces`

System packages:

- `ffmpeg`
- `alsa-utils` (manual microphone test)
- `portaudio19-dev`
- `python3-pyaudio`

Python:

- Python 3.10
- `numpy`
- `openai-whisper`
- `torch` (optional, CUDA auto-detection)
- `pyaudio`
- `pytest` (test only)

Typical setup:

```bash
sudo apt install ffmpeg alsa-utils portaudio19-dev python3-pyaudio
python3 -m pip install -U openai-whisper
```

If `pyaudio` is installed with pip instead of apt, `portaudio19-dev` must be
installed first.

## Microphone Check

Check that Linux can see the input device:

```bash
arecord -l
```

Record and play back a 16 kHz mono sample:

```bash
arecord -f S16_LE -r 16000 -c 1 -d 5 /tmp/mic.wav
aplay /tmp/mic.wav
```

If this fails, fix the OS input device or microphone volume before debugging
Whisper.

## Build

`interfaces` message/service types are required.

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces voice
source install/setup.bash
```

## Test

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select voice --event-handlers console_direct+
colcon test-result --verbose
```

Expected result without local audio samples:

```text
14 passed, 1 skipped
```

The skipped test is the optional Whisper sample regression test. To enable it,
place 16 kHz mono wav files under `test/audio_samples/` and add
`test/audio_samples/manifest.tsv`.

## Run STT Node

Terminal 1, inspect raw STT output:

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /voice/raw_text
```

Terminal 2, run Whisper STT:

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run voice whisper_node --ros-args \
  -p whisper_device:=cuda \
  -p whisper_model_size:=small \
  -p silence_threshold:=0.01 \
  -p max_utterance_seconds:=4.0
```

Use `whisper_device:=cpu` only when CUDA is not available or GPU memory is
reserved for another model. If CUDA is available and the node runs on CPU,
Whisper prints a warning and inference will be slower.

Useful test utterances:

```text
스패너 가져와
십자 드라이버 꺼내줘
복스 소켓 반납해줘
취소
```

## Run Intent Node

`gemma_intent_node` currently uses the deterministic parser. For `fetch` and
`return`, it calls the `db` package's `check_tool_feasibility` service before
publishing `/voice/intent`.

Terminal 1, run DB service:

```bash
ros2 run db db_service_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db
```

Terminal 2, run intent node:

```bash
ros2 run voice gemma_intent_node --ros-args \
  -p require_wake_word:=false
```

Terminal 3, inspect parsed intent:

```bash
ros2 topic echo /voice/intent
```

Terminal 4, either run `whisper_node` and speak, or publish STT text manually:

```bash
ros2 topic pub --once /voice/raw_text std_msgs/msg/String \
  "{data: '스패너 가져와'}"
```

Expected intent fields:

```text
intent_type: fetch
tool_id: spanner_16mm
confidence: 0.65
```

## Parameters

`whisper_node`:

- `enable_microphone`: default `true`
- `whisper_model_size`: default `small`
- `whisper_device`: `auto`, `cuda`, or `cpu`
- `whisper_beam_size`: default `10`
- `whisper_best_of`: default `5`
- `whisper_initial_prompt`: tool and command vocabulary hint
- `max_utterance_seconds`: default `4.0`
- `silence_threshold`: default `0.02`

`gemma_intent_node`:

- `require_wake_word`: default `false`
- `wake_words`: default `["로봇"]`

## Audio Sample Regression Test

Local audio fixtures are intentionally not committed by default. To run the
sample regression test:

```bash
cd /home/thomas/Final_Project/ros2_ws/src/voice/test/audio_samples
printf '%s\t%s\t%s\n' \
  'spanner_fetch_01.wav' 'fetch' 'spanner_16mm' \
  > manifest.tsv
```

Then run:

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
VOICE_SAMPLE_DIR=src/voice/test/audio_samples \
  colcon test --packages-select voice --event-handlers console_direct+
```

Each wav file must be 16 kHz mono and either 16-bit PCM or float32.
