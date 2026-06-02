# voice 패키지

`voice` 패키지는 Track A/B에서 마이크 입력을 Whisper STT로 변환하고,
STT 원문을 프로젝트 표준 `Intent` 메시지로 바꾼다.

전체 흐름:

```text
PyAudio microphone -> whisper_node -> /voice/raw_text
/voice/raw_text -> rule_intent_node -> DB Gate -> /voice/intent
```

`whisper_node`는 원문 텍스트만 publish한다. `fetch`/`return` 가능 여부와
DB Gate 통과 여부는 `rule_intent_node`와 `db` 패키지가 담당한다.
PLC 상태 표시는 `/plc/system_state`에 semantic 문자열을 발행해 연동한다.

## 필요한 라이브러리

ROS2/ament 의존성:

- ROS2 Humble
- `ament_python`
- `rclpy`
- `std_msgs`
- `interfaces`

시스템 패키지:

- `ffmpeg`
- `alsa-utils` - 마이크 수동 테스트용
- `portaudio19-dev`
- `python3-pyaudio`

Python 의존성:

- Python 3.10
- `numpy`
- `openai-whisper`
- `torch` - CUDA 자동 감지용. GPU 사용 시 필요
- `pyaudio`
- `pytest` - 테스트 실행용

일반 설치 예:

```bash
sudo apt install ffmpeg alsa-utils portaudio19-dev python3-pyaudio
python3 -m pip install -U openai-whisper
```

`pyaudio`를 apt가 아니라 pip로 설치할 경우에도 `portaudio19-dev`를 먼저
설치해야 한다.

## 마이크 확인

Linux에서 입력 장치가 보이는지 확인한다.

```bash
arecord -l
```

16 kHz mono로 5초 녹음한 뒤 재생한다.

```bash
arecord -f S16_LE -r 16000 -c 1 -d 5 /tmp/mic.wav
aplay /tmp/mic.wav
```

이 단계에서 녹음이 안 되면 Whisper 문제가 아니라 OS 입력 장치, 마이크 선택,
입력 볼륨 문제를 먼저 해결해야 한다.

## 빌드 방법

`voice` 패키지는 `interfaces`의 message/service 타입을 사용하므로
`interfaces`와 함께 빌드한다.

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces voice
source install/setup.bash
```

## 테스트 방법

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select voice --event-handlers console_direct+
colcon test-result --verbose
```

로컬 오디오 샘플이 없을 때 정상 결과:

```text
20 passed, 1 skipped
```

skip되는 테스트는 선택형 Whisper 샘플 회귀 테스트다. 실행하려면
`test/audio_samples/` 아래에 16 kHz mono wav 파일과 `manifest.tsv`를 둔다.

## STT 노드 실행

Terminal 1 - STT 원문 출력 확인:

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /voice/raw_text
```

Terminal 2 - Whisper STT 실행:

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run voice whisper_node --ros-args \
  -p whisper_device:=cuda \
  -p whisper_model_size:=small \
  -p input_device_name:=pulse \
  -p silence_threshold:=0.01 \
  -p enable_vad:=true \
  -p vad_threshold:=0.5 \
  -p max_utterance_seconds:=4.0
```

CUDA를 사용할 수 없거나 GPU 메모리를 다른 모델이 사용 중일 때만
`whisper_device:=cpu`를 사용한다. CUDA가 가능한데 CPU로 실행하면 Whisper가
경고를 출력하고 추론 속도가 느려진다.

테스트 발화 예:

```text
코봇 스패너 가져와
코봇 십자 드라이버 꺼내줘
코봇 복스 소켓 반납해줘
취소
```

## Intent 노드 실행

`rule_intent_node`는 현재 Gemma 4 없이 deterministic parser를 사용한다.
`fetch`와 `return` 명령은 `/voice/intent`로 publish되기 전에 `db` 패키지의
`/db/CheckToolFeasibility` 서비스를 통과해야 한다.
음성 흐름 중 PLC에는 다음 상태가 발행된다.

| 노드 | 상황 | `/plc/system_state` |
|------|------|----------------------|
| `whisper_node` | STT 원문 publish | `listening` |
| `rule_intent_node` | wake word 통과 후 명령 해석/DB Gate 확인 | `inferring` |
| `rule_intent_node` | cancel/unknown 처리 완료 | `idle` |
| `rule_intent_node` | DB 서비스 없음/DB Gate 실패/거부 | `error` |

`fetch`/`return`이 DB Gate를 통과한 뒤에는 downstream motion 또는
`db` 패키지의 `intent_status_simulator_node`가 `moving`, `idle`, `error`를
이어 받는다.

Terminal 1 - DB 서비스 실행:

```bash
ros2 run db db_service_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db
```

Terminal 2 - intent 노드 실행:

```bash
ros2 run voice rule_intent_node --ros-args \
  -p require_wake_word:=true \
  -p wake_words:="[코봇]"
```

Terminal 3 - 파싱된 intent 확인:

```bash
ros2 topic echo /voice/intent
```

Terminal 4 - PLC semantic 상태 확인:

```bash
ros2 topic echo /plc/system_state
```

Terminal 5 - 실제 마이크 대신 STT 원문을 직접 발행:

```bash
ros2 topic pub --once /voice/raw_text std_msgs/msg/String \
  "{data: '코봇 스패너 가져와'}"
```

예상 intent:

```text
intent_type: fetch
tool_id: spanner_16mm
confidence: 0.65
```

이때 `/plc/system_state`에는 먼저 `inferring`이 찍힌다. DB Gate가 거부하거나
서비스가 없으면 `error`, cancel/unknown 계열이면 `idle`로 돌아간다.

## 주요 파라미터

`whisper_node`:

- `enable_microphone`: 기본값 `true`
- `whisper_model_size`: 기본값 `small`
- `whisper_device`: `auto`, `cuda`, `cpu`
- `whisper_beam_size`: 기본값 `10`
- `whisper_best_of`: 기본값 `5`
- `whisper_initial_prompt`: 공구명과 명령어 vocabulary 힌트
- `input_device_index`: 기본값 `-1` (PyAudio 기본 입력 장치 사용)
- `input_device_name`: 기본값 `""`, 예: `pulse`, `USB`
- `max_utterance_seconds`: 기본값 `4.0`
- `silence_threshold`: 기본값 `0.02`
- `min_speech_chunks`: 기본값 `4`
- `enable_vad`: 기본값 `true`
- `vad_threshold`: 기본값 `0.5`
- `vad_min_speech_ms`: 기본값 `250`
- `vad_min_silence_ms`: 기본값 `100`
- `vad_speech_pad_ms`: 기본값 `100`

`rule_intent_node`:

- `require_wake_word`: 기본값 `true`
- `wake_words`: 기본값 `["코봇"]`

VAD를 켠 상태로 마이크를 실행하려면 Python 환경에 Silero VAD가 필요하다.

```bash
pip install silero-vad
```

## 오디오 샘플 회귀 테스트

로컬 오디오 fixture는 기본적으로 commit하지 않는다. 샘플 회귀 테스트를
실행하려면 아래처럼 `manifest.tsv`를 만든다.

```bash
cd /home/thomas/Final_Project/ros2_ws/src/voice/test/audio_samples
printf '%s\t%s\t%s\n' \
  'spanner_fetch_01.wav' 'fetch' 'spanner_16mm' \
  > manifest.tsv
```

그 다음 테스트를 실행한다.

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
VOICE_SAMPLE_DIR=src/voice/test/audio_samples \
  colcon test --packages-select voice --event-handlers console_direct+
```

각 wav 파일은 16 kHz mono여야 하며, 16-bit PCM 또는 float32 형식을 사용한다.
