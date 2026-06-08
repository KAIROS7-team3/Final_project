# voice 패키지

`voice` 패키지는 Track A/B에서 마이크 입력을 Whisper STT로 변환하고,
STT 원문을 프로젝트 표준 `Intent` 메시지로 바꾼다.

전체 흐름:

```text
PyAudio microphone -> whisper_node -> /voice/raw_text
/voice/raw_text -> gemma_intent_node -> DB Gate -> /voice/intent
```

`whisper_node`는 원문 텍스트만 publish한다. `fetch`/`return` 가능 여부와
DB Gate 통과 여부는 `gemma_intent_node`와 `db` 패키지가 담당한다.

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
  -p silence_threshold:=0.01 \
  -p max_utterance_seconds:=4.0
```

CUDA를 사용할 수 없거나 GPU 메모리를 다른 모델이 사용 중일 때만
`whisper_device:=cpu`를 사용한다. CUDA가 가능한데 CPU로 실행하면 Whisper가
경고를 출력하고 추론 속도가 느려진다.

테스트 발화 예:

```text
스패너 가져와
십자 드라이버 꺼내줘
복스 소켓 반납해줘
취소
```

## Intent 노드 실행

`gemma_intent_node`는 Track A/B의 기본 intent 경로다. 로컬 Gemma 모델로
`fetch`/`return`/`cancel`/`unknown`을 분류하고, `fetch`와 `return`은
`db` 패키지의 `/db/CheckToolFeasibility` 서비스를 통과해야 한다.

## Voice launch

Whisper STT와 Gemma intent 노드를 한 번에 띄우려면 아래 launch를 사용한다.
DB feasibility gate는 별도 `db_service_node`가 필요하므로, DB 서비스는
다른 터미널이나 상위 launch에서 먼저 실행한다.

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch voice voice.launch.py
```

자주 조정하는 값:

```bash
ros2 launch voice voice.launch.py \
  enable_microphone:=false \
  whisper_device:=cpu \
  require_wake_word:=true \
  gemma_device:=auto
```

`enable_microphone:=false`로 두면 실제 마이크 대신 `/voice/raw_text`를 직접
발행해서 intent 분류를 시험할 수 있다.

Terminal 1 - DB 서비스 실행:

```bash
ros2 run db db_service_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db
```

Terminal 2 - intent 노드 실행:

```bash
ros2 run voice gemma_intent_node --ros-args \
  -p require_wake_word:=true \
  -p wake_words:='["코봇"]'
```

Terminal 3 - 파싱된 intent 확인:

```bash
ros2 topic echo /voice/intent
```

Terminal 4 - 실제 마이크 대신 STT 원문을 직접 발행:

```bash
ros2 topic pub --once /voice/raw_text std_msgs/msg/String \
  "{data: '스패너 가져와'}"
```

예상 intent:

```text
intent_type: fetch
tool_id: spanner_16mm
confidence: 0.65
```

## 주요 파라미터

`whisper_node`:

- `enable_microphone`: 기본값 `true`
- `whisper_model_size`: 기본값 `small`
- `whisper_device`: `auto`, `cuda`, `cpu`
- `whisper_beam_size`: 기본값 `10`
- `whisper_best_of`: 기본값 `5`
- `whisper_initial_prompt`: 공구명과 명령어 vocabulary 힌트
- `max_utterance_seconds`: 기본값 `4.0`
- `silence_threshold`: 기본값 `0.02`

`gemma_intent_node`:

- `require_wake_word`: 기본값 `true`
- `wake_words`: 기본값 `["코봇", "코 봇", "코버", "코 버", "고봇", "고 봇", "고버", "고 버", "코벗", "코 벗", "고벗", "고 벗", "코보트", "코 보트", "고보트", "고 보트"]`
- `gemma_model_id`: 기본값 `/home/thomas/models/gemma/gemma-3-1b-it`
- `gemma_device`: `auto`, `cuda`, `cpu`
- `gemma_confidence_threshold`: 기본값 `0.75`
- `gemma_max_new_tokens`: 기본값 `128`
- `gemma_temperature`: 기본값 `0.0`
- `gemma_warmup`: 기본값 `true`

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
