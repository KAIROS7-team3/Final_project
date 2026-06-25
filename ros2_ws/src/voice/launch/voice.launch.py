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

    voice_share = Path(get_package_share_directory("voice"))
    gemma_config_path = voice_share / "config" / "gemma.yaml"
    gemma_prompt_template_path = voice_share / "gemma_prompt.txt"
    toolbox_path = voice_share / "config" / "toolbox.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_microphone", default_value="true"),
            DeclareLaunchArgument(
                "whisper_backend",
                default_value="faster",
                description="STT 백엔드: faster (faster-whisper, VAD 내장) 또는 openai",
            ),
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
                default_value="",
            ),
            DeclareLaunchArgument(
                "follow_up_on_wake_word_only",
                default_value="false",
                description="(Option C 비활성 시만 사용) 웨이크워드만 발화 시 즉시 후속 녹음",
            ),
            DeclareLaunchArgument(
                "external_followup_control",
                default_value="true",
                description="True면 whisper keyword follow-up 비활성 — gemma_intent_node가 담당",
            ),
            DeclareLaunchArgument(
                "followup_max_retries",
                default_value="2",
                description="Gemma unknown 시 후속 발화 최대 재시도 횟수",
            ),
            DeclareLaunchArgument(
                "followup_context_timeout",
                default_value="8.0",
                description="후속 발화 대기 타임아웃(초) — 초과 시 컨텍스트 초기화",
            ),
            DeclareLaunchArgument("max_utterance_seconds", default_value="4.0"),
            DeclareLaunchArgument("silence_threshold", default_value="0.02"),
            DeclareLaunchArgument(
                "reject_hallucinated_transcripts", default_value="true"
            ),
            DeclareLaunchArgument("require_wake_word", default_value="true"),
            DeclareLaunchArgument(
                "gemma_model_id",
                default_value="~/models/gemma/gemma-3-1b-it",
            ),
            DeclareLaunchArgument(
                "gemma_prompt_template_path",
                default_value=str(gemma_prompt_template_path),
            ),
            DeclareLaunchArgument(
                "toolbox_path",
                default_value=str(toolbox_path),
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
                        "whisper_backend": LaunchConfiguration("whisper_backend"),
                        "follow_up_on_wake_word_only": ParameterValue(
                            LaunchConfiguration("follow_up_on_wake_word_only"),
                            value_type=bool,
                        ),
                        "external_followup_control": ParameterValue(
                            LaunchConfiguration("external_followup_control"),
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
                        "gemma_prompt_template_path": LaunchConfiguration(
                            "gemma_prompt_template_path"
                        ),
                        "toolbox_path": LaunchConfiguration("toolbox_path"),
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
                        "followup_max_retries": ParameterValue(
                            LaunchConfiguration("followup_max_retries"),
                            value_type=int,
                        ),
                        "followup_context_timeout": ParameterValue(
                            LaunchConfiguration("followup_context_timeout"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
