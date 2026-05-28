"""Vision pipeline 통합 런치 파일 (Track A/B).

D455f 카메라 + 비전 파이프라인 전체(yolo → pose → tracker → context_builder)를
단일 명령으로 기동한다.

사용:
  ros2 launch vision vision_pipeline.launch.py
  ros2 launch vision vision_pipeline.launch.py debug:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="publish_annotated_image 여부 (config/vision.yaml 설정값 우선)",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ]),
        launch_arguments={
            "camera_namespace": "d455f",
            "camera_name": "d455f",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "848x480x30",
            "pointcloud.enable": "false",
        }.items(),
    )

    yolo_node = Node(
        package="vision",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
    )

    pose_node = Node(
        package="vision",
        executable="pose_node",
        name="pose_node",
        output="screen",
    )

    tracker_node = Node(
        package="vision",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
    )

    context_builder = Node(
        package="vision",
        executable="context_builder",
        name="context_builder",
        output="screen",
    )

    return LaunchDescription([
        debug_arg,
        realsense_launch,
        yolo_node,
        pose_node,
        tracker_node,
        context_builder,
    ])
