"""Vision pipeline 통합 런치 파일 (Track A/B).

D455f(탑뷰) + C270(그리퍼) 두 카메라와 비전 파이프라인 전체를 단일 명령으로 기동한다.
yolo_node는 camera_type 파라미터로 구분된 두 인스턴스로 각각 기동된다.
  - yolo_node_top_view : camera_type=top_view, /d455f/color/image_raw 구독
  - yolo_node_gripper  : camera_type=gripper,  /c270/image_raw 구독

사용:
  ros2 launch vision vision_pipeline.launch.py
  ros2 launch vision vision_pipeline.launch.py debug:=false
  ros2 launch vision vision_pipeline.launch.py c270_device:=/dev/video4
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
        default_value="/dev/c270",
        description="C270 그리퍼 캠 V4L2 디바이스 경로 (udev 심링크 /dev/c270, 없으면 /dev/video4 직접 지정)",
    )
    top_view_device_arg = DeclareLaunchArgument(
        "top_view_device",
        default_value="",
        description="탑뷰 YOLO device 오버라이드 (예: cpu, cuda). 빈 값이면 vision.yaml 설정 사용",
    )
    gripper_device_arg = DeclareLaunchArgument(
        "gripper_device",
        default_value="",
        description="그리퍼 YOLO device 오버라이드 (예: cpu, cuda). 빈 값이면 vision.yaml 설정 사용",
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
        parameters=[{"camera_type": "top_view", "device": LaunchConfiguration("top_view_device")}],
    )

    yolo_node_gripper = Node(
        package="vision",
        executable="yolo_node",
        name="yolo_node_gripper",
        output="screen",
        parameters=[{"camera_type": "gripper", "device": LaunchConfiguration("gripper_device")}],
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

    # gripper_marker_scan_node: yolo_node_gripper 마스크 + ArUco 마커로
    # /vision/tool_gripper_pose (PoseStamped, base_link frame) 발행.
    # 스캔 BT의 CollectAndSave가 이 토픽을 소비한다.
    gripper_marker_scan = Node(
        package="vision",
        executable="gripper_marker_scan_node",
        name="gripper_marker_scan_node",
        output="screen",
    )

    return LaunchDescription([
        debug_arg,
        c270_device_arg,
        top_view_device_arg,
        gripper_device_arg,
        c270_node,
        realsense_launch,
        yolo_node_top_view,
        yolo_node_gripper,
        pose_node,
        tracker_node,
        context_builder,
        gripper_marker_scan,
    ])
