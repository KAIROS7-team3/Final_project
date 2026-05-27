"""Hand-eye 캘리브레이션 런치 (eye-to-hand, D455f + e0509).

사전 조건:
  - easy_handeye2 빌드 완료 (scripts/calibrate_hand_eye.sh §1 참조)
  - doosan-robot2 드라이버 실행 중 (담당: B)
  - CharUco 보드 인쇄 완료 (scripts/calibrate_hand_eye.sh §0 참조)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ]),
        launch_arguments={
            "camera_namespace": "d455f",
            "camera_name": "d455f",
            "enable_color": "true",
            "enable_depth": "false",
            "align_depth.enable": "false",
            "rgb_camera.color_profile": "1280x720x30",
        }.items(),
    )

    # easy_handeye2 캘리브레이션 노드 (eye-to-hand)
    handeye_node = Node(
        package="easy_handeye2",
        executable="calibrate",
        name="handeye_calibration",
        parameters=[{
            "name": "d455f_e0509",
            "eye_on_hand": False,
            "robot_base_frame": "base_link",
            "robot_effector_frame": "tool0",
            "tracking_base_frame": "d455f_color_optical_frame",
            "tracking_marker_frame": "aruco_marker_frame",
            "publish_tf": True,
        }],
        output="screen",
    )

    return LaunchDescription([realsense_launch, handeye_node])
