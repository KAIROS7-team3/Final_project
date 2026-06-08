"""Compatibility wrapper for the previous no-Gemma voice launch path.

This wrapper keeps stale build/install symlinks from breaking while the
Gemma-based `voice.launch.py` is the primary launch entrypoint.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Delegate to the Gemma-based voice launch."""

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("voice"), "launch", "voice.launch.py"]
                    )
                )
            )
        ]
    )
