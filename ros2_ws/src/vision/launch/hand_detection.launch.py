"""핸드 감지 파이프라인 런치 파일.

mediapipe_hands_node + hand_node + hand_viz_node 세 노드를 단일 명령으로 기동한다.
RealSense(터미널 2)는 별도로 먼저 실행해야 한다.

사용:
  source ~/Final_project/ros2_ws/install/setup.bash
  ros2 launch vision hand_detection.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    # vision_pipeline.launch(namespace="") → single /d455f/...  (기본값)
    # realsense_bringup.launch(namespace=d455f) 사용 시 double 경로로 override:
    #   depth_topic:=/d455f/d455f/aligned_depth_to_color/image_raw
    #   image_topic:=/d455f/d455f/color/image_raw
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/d455f/color/image_raw",
        description="컬러 이미지 토픽 (realsense camera_namespace에 맞춰 지정)",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/d455f/aligned_depth_to_color/image_raw",
        description="aligned depth 토픽 (realsense camera_namespace에 맞춰 지정)",
    )

    # mediapipe는 자체 protobuf 4.x 필요 → 전용 venv(handpose_venv)에서 격리 실행.
    # venv numpy는 1.26.4 고정 (mediapipe·cv_bridge·cv2 모두 numpy 1.x ABI). README 참고.
    mediapipe_node = ExecuteProcess(
        cmd=[
            "/home/user/Final_project/ros2_ws/src/handpose_ros/scripts/mediapipe_hands_wrapper.sh",
            "--ros-args",
            "-r", "__node:=mediapipe_hands_node",
            "-p", ["image_topic:=", LaunchConfiguration("image_topic")],
            "-p", ["flip_image:=", LaunchConfiguration("flip_image")],
            "-p", ["min_detection_confidence:=", LaunchConfiguration("min_detection_confidence")],
            "-p", ["max_num_hands:=", LaunchConfiguration("max_num_hands")],
        ],
        output="screen",
        name="mediapipe_hands_node",
    )

    hand_node = Node(
        package="vision",
        executable="hand_node",
        name="hand_node",
        output="screen",
        parameters=[{"depth_topic": LaunchConfiguration("depth_topic")}],
    )

    # hand_viz_node는 대시보드 탑뷰 "핸드오버" 탭으로 대체 — 별도 cv2 창 불필요
    # hand_viz_node = Node(
    #     package="vision",
    #     executable="hand_viz_node",
    #     name="hand_viz_node",
    #     output="screen",
    # )

    return LaunchDescription([
        flip_arg,
        confidence_arg,
        max_hands_arg,
        image_topic_arg,
        depth_topic_arg,
        mediapipe_node,
        hand_node,
        # hand_viz_node,
    ])
