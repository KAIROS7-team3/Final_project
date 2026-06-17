from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pointcloud_arg = DeclareLaunchArgument(
        'pointcloud',
        default_value='false',
        description='pointcloud 토픽 활성화 여부 — 캘리브레이션 시에만 true',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ]),
        launch_arguments={
            "camera_namespace": "d455f",
            "camera_name": "d455f",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "848x480x30",
            "pointcloud.enable": LaunchConfiguration('pointcloud'),
        }.items(),
    )

    camera_node = Node(
        package="vision",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )

    return LaunchDescription([pointcloud_arg, realsense_launch, camera_node])
