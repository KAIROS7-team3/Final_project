"""Gemma 4 E2B 오디오 직접 분류 launch.

Whisper STT 없이 마이크 오디오를 Gemma 4 E2B에 바로 넣어 의도를 분류한다.
기존 voice.launch.py 파이프라인과 병행 사용 가능하다 (같은 /voice/intent 토픽).

사용:
  ros2 launch voice voice_gemma4_audio.launch.py
  ros2 launch voice voice_gemma4_audio.launch.py require_wake_word:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    voice_share = Path(get_package_share_directory("voice"))
    toolbox_path = voice_share / "config" / "toolbox.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_microphone", default_value="true"),
            DeclareLaunchArgument(
                "gemma4_model_id",
                default_value="~/models/gemma/gemma-4-e2b-it",
            ),
            DeclareLaunchArgument("gemma4_device", default_value="auto"),
            DeclareLaunchArgument(
                "gemma4_confidence_threshold", default_value="0.85"
            ),
            DeclareLaunchArgument("gemma4_max_new_tokens", default_value="96"),
            DeclareLaunchArgument("gemma4_warmup", default_value="true"),
            DeclareLaunchArgument(
                "toolbox_path", default_value=str(toolbox_path)
            ),
            DeclareLaunchArgument("require_wake_word", default_value="true"),
            DeclareLaunchArgument("max_utterance_seconds", default_value="5.0"),
            DeclareLaunchArgument("silence_threshold", default_value="0.02"),
            Node(
                package="voice",
                executable="gemma4_audio_node",
                name="gemma4_audio_node",
                output="screen",
                parameters=[
                    {
                        "enable_microphone": ParameterValue(
                            LaunchConfiguration("enable_microphone"),
                            value_type=bool,
                        ),
                        "gemma4_model_id": LaunchConfiguration("gemma4_model_id"),
                        "gemma4_device": LaunchConfiguration("gemma4_device"),
                        "gemma4_confidence_threshold": ParameterValue(
                            LaunchConfiguration("gemma4_confidence_threshold"),
                            value_type=float,
                        ),
                        "gemma4_max_new_tokens": ParameterValue(
                            LaunchConfiguration("gemma4_max_new_tokens"),
                            value_type=int,
                        ),
                        "gemma4_warmup": ParameterValue(
                            LaunchConfiguration("gemma4_warmup"),
                            value_type=bool,
                        ),
                        "toolbox_path": LaunchConfiguration("toolbox_path"),
                        "require_wake_word": ParameterValue(
                            LaunchConfiguration("require_wake_word"),
                            value_type=bool,
                        ),
                        "max_utterance_seconds": ParameterValue(
                            LaunchConfiguration("max_utterance_seconds"),
                            value_type=float,
                        ),
                        "silence_threshold": ParameterValue(
                            LaunchConfiguration("silence_threshold"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
