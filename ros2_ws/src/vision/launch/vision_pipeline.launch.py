"""Vision pipeline 통합 런치 파일 (Track A/B).

D455f(탑뷰) + C270(그리퍼) 두 카메라와 비전 파이프라인 전체를 단일 명령으로 기동한다.
yolo_node는 camera_type 파라미터로 구분된 두 인스턴스로 각각 기동된다.
  - yolo_node_top_view : camera_type=top_view, /d455f/color/image_raw 구독
  - yolo_node_gripper  : camera_type=gripper,  /c270/image_raw 구독

사용:
  ros2 launch vision vision_pipeline.launch.py
  ros2 launch vision vision_pipeline.launch.py debug:=false
  ros2 launch vision vision_pipeline.launch.py c270_device:=/dev/video2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    c270_device_arg = DeclareLaunchArgument(
        "c270_device",
        default_value="/dev/video2",
        description="C270 그리퍼 캠 V4L2 디바이스 경로",
    )

    c270_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="c270_camera",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("c270_device"),
            "image_size":   [640, 480],
            "camera_frame_id": "gripper_cam_link",
        }],
        remappings=[("/image_raw", "/c270/image_raw")],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ]),
        launch_arguments={
            "camera_namespace": "",
            "camera_name": "d455f",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "848x480x30",
            "pointcloud.enable": "false",
        }.items(),
    )

    yolo_node_top_view = Node(
        package="vision",
        executable="yolo_node",
        name="yolo_node_top_view",
        output="screen",
        parameters=[{"camera_type": "top_view"}],
    )

    yolo_node_gripper = Node(
        package="vision",
        executable="yolo_node",
        name="yolo_node_gripper",
        output="screen",
        parameters=[{"camera_type": "gripper"}],
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
        c270_device_arg,
        c270_node,
        realsense_launch,
        yolo_node_top_view,
        yolo_node_gripper,
        pose_node,
        tracker_node,
        context_builder,
    ])
