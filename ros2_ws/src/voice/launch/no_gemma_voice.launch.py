"""Launch the No-Gemma voice pipeline for Track A/B."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("voice"), "config", "no_gemma.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameters for the No-Gemma voice pipeline.",
            ),
            Node(
                package="voice",
                executable="whisper_node",
                name="whisper_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="voice",
                executable="rule_intent_node",
                name="rule_intent_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
