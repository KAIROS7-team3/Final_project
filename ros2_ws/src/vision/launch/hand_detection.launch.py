"""핸드 감지 파이프라인 런치 파일.

mediapipe_hands_node + hand_node + hand_viz_node 세 노드를 단일 명령으로 기동한다.
RealSense(터미널 2)는 별도로 먼저 실행해야 한다.

사용:
  source ~/Final_project/ros2_ws/install/setup.bash
  ros2 launch vision hand_detection.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    flip_arg = DeclareLaunchArgument(
        "flip_image", default_value="false", description="이미지 좌우 반전 여부"
    )
    confidence_arg = DeclareLaunchArgument(
        "min_detection_confidence", default_value="0.7", description="MediaPipe 감지 최소 신뢰도"
    )
    max_hands_arg = DeclareLaunchArgument(
        "max_num_hands", default_value="4", description="최대 감지 손 개수"
    )

    mediapipe_node = Node(
        package="handpose_ros",
        executable="mediapipe_hands_node",
        name="mediapipe_hands_node",
        output="screen",
        parameters=[{
            "flip_image": LaunchConfiguration("flip_image"),
            "min_detection_confidence": LaunchConfiguration("min_detection_confidence"),
            "max_num_hands": LaunchConfiguration("max_num_hands"),
        }],
    )

    hand_node = Node(
        package="vision",
        executable="hand_node",
        name="hand_node",
        output="screen",
    )

    hand_viz_node = Node(
        package="vision",
        executable="hand_viz_node",
        name="hand_viz_node",
        output="screen",
    )

    return LaunchDescription([
        flip_arg,
        confidence_arg,
        max_hands_arg,
        mediapipe_node,
        hand_node,
        hand_viz_node,
    ])
