"""demo.launch.py
─────────────────
데모 전용 런치 — system.launch.py를 포함하고 demo_trigger 노드를 추가한다.

프로덕션 런치는 orchestrator/launch/system.launch.py 를 직접 사용한다.
여기에 demo_trigger 등 수동 입력 도구가 필요할 때만 이 파일을 사용한다.

실행 예:
  ros2 launch demo demo.launch.py
  ros2 launch demo demo.launch.py mode:=real robot_ip:=110.120.1.38
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument("mode",      default_value="virtual"),
        DeclareLaunchArgument("robot_ip",  default_value="110.120.1.38"),
        DeclareLaunchArgument("robot_ns",  default_value="dsr01"),
        DeclareLaunchArgument("voice",     default_value="false"),
        DeclareLaunchArgument("plc",       default_value="false"),
        DeclareLaunchArgument("plc_port",  default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        DeclareLaunchArgument("db_path",   default_value="~/robot_tools.db"),
    ]

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orchestrator"),
                "launch",
                "system.launch.py",
            ])
        ]),
        launch_arguments={
            "mode":      LaunchConfiguration("mode"),
            "robot_ip":  LaunchConfiguration("robot_ip"),
            "robot_ns":  LaunchConfiguration("robot_ns"),
            "voice":     LaunchConfiguration("voice"),
            "plc":       LaunchConfiguration("plc"),
            "plc_port":  LaunchConfiguration("plc_port"),
            "dashboard": LaunchConfiguration("dashboard"),
            "db_path":   LaunchConfiguration("db_path"),
        }.items(),
    )

    return LaunchDescription(args + [system])
