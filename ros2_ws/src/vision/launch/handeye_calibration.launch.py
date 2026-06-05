"""Hand-eye 캘리브레이션 런치 (eye-to-hand, D455f + e0509).

사전 조건:
  - doosan-robot2 드라이버 실행 중
  - ArUco 마커 TCP에 부착
  (RealSense는 이 런치에서 pointcloud:=true 로 함께 기동됨)

오버라이드 예시:
  ros2 launch vision handeye_calibration.launch.py robot_effector_frame:=tool0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='base_link',
    )
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame', default_value='link_6',
    )
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame', default_value='d455f_color_optical_frame',
    )
    tracking_marker_frame_arg = DeclareLaunchArgument(
        'tracking_marker_frame', default_value='aruco_marker_frame',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('vision'), 'launch', 'realsense_bringup.launch.py'])
        ]),
        launch_arguments={'pointcloud': 'true'}.items(),
    )

    # easy_handeye2 캘리브레이션 서버 (eye-to-hand)
    handeye_node = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_calibration',
        parameters=[{
            'name': 'd455f_e0509',
            'calibration_type': 'eye_to_hand',
            'eye_on_hand': False,
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'publish_tf': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        realsense_launch,
        handeye_node,
    ])
