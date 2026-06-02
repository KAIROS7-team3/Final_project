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
            "enable_depth": "true",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "848x480x30",
            "pointcloud.enable": "true",
        }.items(),
    )

    camera_node = Node(
        package="vision",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )

    return LaunchDescription([realsense_launch, camera_node])
