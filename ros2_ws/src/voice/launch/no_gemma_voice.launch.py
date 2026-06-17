"""Fallback voice launch without Gemma.

이 launch는 Whisper STT와 deterministic `rule_intent_node`를 함께 올린다.
Gemma 모델이 없거나 baseline 파서로만 검증하고 싶을 때 사용한다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Whisper STT와 rule-based intent node를 함께 실행한다."""

    voice_share = Path(get_package_share_directory("voice"))
    no_gemma_config_path = voice_share / "config" / "no_gemma.yaml"

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
            Node(
                package="voice",
                executable="whisper_node",
                name="whisper_node",
                output="screen",
                parameters=[
                    str(no_gemma_config_path),
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
                    },
                ],
            ),
            Node(
                package="voice",
                executable="rule_intent_node",
                name="rule_intent_node",
                output="screen",
                parameters=[str(no_gemma_config_path)],
            ),
        ]
    )

