"""Voice STT + Gemma intent launch file.

이 launch는 `whisper_node`와 `gemma_intent_node`를 한 번에 올린다.
DB feasibility gate는 별도 `db_service_node`가 제공해야 하므로, DB 서비스는
상위 launch나 별도 터미널에서 먼저 띄운다.

사용:
  ros2 launch voice voice.launch.py
  ros2 launch voice voice.launch.py enable_microphone:=false whisper_device:=cpu
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Whisper STT와 Gemma intent 노드를 함께 실행한다."""

    gemma_config_path = (
        Path(get_package_share_directory("voice")) / "config" / "gemma.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_microphone", default_value="true"),
            DeclareLaunchArgument("whisper_device", default_value="auto"),
            DeclareLaunchArgument("whisper_model_size", default_value="small"),
            DeclareLaunchArgument("whisper_beam_size", default_value="10"),
            DeclareLaunchArgument("whisper_best_of", default_value="5"),
            DeclareLaunchArgument(
                "whisper_no_speech_threshold", default_value="0.6"
            ),
            DeclareLaunchArgument(
                "whisper_logprob_threshold", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "whisper_compression_ratio_threshold", default_value="2.4"
            ),
            DeclareLaunchArgument(
                "whisper_initial_prompt",
                default_value=(
                    "코봇, 코버, 코보, 코부, 고봇, 고버, 고보, 고부, 꼬부, "
                    "공구함, 두산 로봇, 스테이징, 십자 드라이버, 커터칼, "
                    "라쳇 렌치, 멕가이버, 스패너 16mm, 복스 소켓 19mm, "
                    "가져와, 꺼내줘, 반납, 돌려놔, 취소"
                ),
            ),
            DeclareLaunchArgument("max_utterance_seconds", default_value="4.0"),
            DeclareLaunchArgument("silence_threshold", default_value="0.02"),
            DeclareLaunchArgument(
                "reject_hallucinated_transcripts", default_value="true"
            ),
            DeclareLaunchArgument("require_wake_word", default_value="true"),
            DeclareLaunchArgument(
                "gemma_model_id",
                default_value="gemma-4-local",
            ),
            DeclareLaunchArgument("gemma_device", default_value="auto"),
            DeclareLaunchArgument(
                "gemma_confidence_threshold",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "gemma_max_new_tokens",
                default_value="128",
            ),
            DeclareLaunchArgument("gemma_temperature", default_value="0.0"),
            DeclareLaunchArgument("gemma_warmup", default_value="true"),
            Node(
                package="voice",
                executable="whisper_node",
                name="whisper_node",
                output="screen",
                parameters=[
                    {
                        "enable_microphone": ParameterValue(
                            LaunchConfiguration("enable_microphone"),
                            value_type=bool,
                        ),
                        "whisper_device": LaunchConfiguration("whisper_device"),
                        "whisper_model_size": LaunchConfiguration(
                            "whisper_model_size"
                        ),
                        "whisper_beam_size": ParameterValue(
                            LaunchConfiguration("whisper_beam_size"),
                            value_type=int,
                        ),
                        "whisper_best_of": ParameterValue(
                            LaunchConfiguration("whisper_best_of"),
                            value_type=int,
                        ),
                        "whisper_initial_prompt": LaunchConfiguration(
                            "whisper_initial_prompt"
                        ),
                        "whisper_no_speech_threshold": ParameterValue(
                            LaunchConfiguration("whisper_no_speech_threshold"),
                            value_type=float,
                        ),
                        "whisper_logprob_threshold": ParameterValue(
                            LaunchConfiguration("whisper_logprob_threshold"),
                            value_type=float,
                        ),
                        "whisper_compression_ratio_threshold": ParameterValue(
                            LaunchConfiguration(
                                "whisper_compression_ratio_threshold"
                            ),
                            value_type=float,
                        ),
                        "max_utterance_seconds": ParameterValue(
                            LaunchConfiguration("max_utterance_seconds"),
                            value_type=float,
                        ),
                        "silence_threshold": ParameterValue(
                            LaunchConfiguration("silence_threshold"),
                            value_type=float,
                        ),
                        "reject_hallucinated_transcripts": ParameterValue(
                            LaunchConfiguration(
                                "reject_hallucinated_transcripts"
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="voice",
                executable="gemma_intent_node",
                name="gemma_intent_node",
                output="screen",
                parameters=[
                    str(gemma_config_path),
                    {
                        "require_wake_word": ParameterValue(
                            LaunchConfiguration("require_wake_word"),
                            value_type=bool,
                        ),
                        "gemma_model_id": LaunchConfiguration("gemma_model_id"),
                        "gemma_device": LaunchConfiguration("gemma_device"),
                        "gemma_confidence_threshold": ParameterValue(
                            LaunchConfiguration("gemma_confidence_threshold"),
                            value_type=float,
                        ),
                        "gemma_max_new_tokens": ParameterValue(
                            LaunchConfiguration("gemma_max_new_tokens"),
                            value_type=int,
                        ),
                        "gemma_temperature": ParameterValue(
                            LaunchConfiguration("gemma_temperature"),
                            value_type=float,
                        ),
                        "gemma_warmup": ParameterValue(
                            LaunchConfiguration("gemma_warmup"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
