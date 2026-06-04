"""Hand-eye 캘리브레이션 런치 (eye-to-hand, D455f + e0509).

사전 조건:
  - realsense_bringup.launch.py 별도 터미널에서 실행 중
  - doosan-robot2 드라이버 실행 중
  - ArUco 마커 TCP에 부착
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # easy_handeye2 캘리브레이션 서버 (eye-to-hand)
    handeye_node = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_calibration',
        parameters=[{
            'name': 'd455f_e0509',
            'calibration_type': 'eye_to_hand',
            'eye_on_hand': False,
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'link_6',
            'tracking_base_frame': 'd455f_d455f_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker_frame',
            'publish_tf': True,
        }],
        output='screen',
    )

    return LaunchDescription([handeye_node])
