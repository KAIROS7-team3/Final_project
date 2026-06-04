"""PLC 노드 실행용 launch file.

기본 설정은 패키지 share 디렉터리에 설치된 `config/xgb_plc.yaml`을 읽고,
현장마다 달라지는 serial port/baudrate/device_id는 launch argument로 덮어쓴다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """`plc_node`를 설정 파일과 launch override parameter로 실행한다."""

    config_path = Path(get_package_share_directory("plc")) / "config" / "xgb_plc.yaml"

    return LaunchDescription(
        [
            # USB serial 장치명은 PC/udev 설정에 따라 달라질 수 있으므로 launch에서
            # 바로 바꿀 수 있게 둔다.
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("device_id", default_value="1"),
            DeclareLaunchArgument("reset_coil_address", default_value="256"),
            DeclareLaunchArgument("pulse_duration_s", default_value="0.2"),
            DeclareLaunchArgument("enable_watchdog", default_value="false"),
            DeclareLaunchArgument("watchdog_coil_address", default_value="-1"),
            DeclareLaunchArgument("watchdog_period_s", default_value="0.25"),
            DeclareLaunchArgument("enable_estop_poll", default_value="false"),
            DeclareLaunchArgument("estop_input_address", default_value="-1"),
            DeclareLaunchArgument("estop_poll_period_s", default_value="0.1"),
            DeclareLaunchArgument("db_path", default_value="robot_arm.db"),
            Node(
                package="plc",
                executable="plc_node",
                name="plc_node",
                output="screen",
                parameters=[
                    # YAML 기본값을 먼저 로드한 뒤, 아래 dict로 launch argument 값을
                    # 덮어쓴다. 숫자 parameter는 문자열로 들어오므로 ParameterValue로
                    # 타입을 명시한다.
                    str(config_path),
                    {
                        "port": LaunchConfiguration("port"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"), value_type=int
                        ),
                        "device_id": ParameterValue(
                            LaunchConfiguration("device_id"), value_type=int
                        ),
                        "reset_coil_address": ParameterValue(
                            LaunchConfiguration("reset_coil_address"), value_type=int
                        ),
                        "pulse_duration_s": ParameterValue(
                            LaunchConfiguration("pulse_duration_s"), value_type=float
                        ),
                        "enable_watchdog": ParameterValue(
                            LaunchConfiguration("enable_watchdog"), value_type=bool
                        ),
                        "watchdog_coil_address": ParameterValue(
                            LaunchConfiguration("watchdog_coil_address"),
                            value_type=int,
                        ),
                        "watchdog_period_s": ParameterValue(
                            LaunchConfiguration("watchdog_period_s"),
                            value_type=float,
                        ),
                        "enable_estop_poll": ParameterValue(
                            LaunchConfiguration("enable_estop_poll"), value_type=bool
                        ),
                        "estop_input_address": ParameterValue(
                            LaunchConfiguration("estop_input_address"), value_type=int
                        ),
                        "estop_poll_period_s": ParameterValue(
                            LaunchConfiguration("estop_poll_period_s"),
                            value_type=float,
                        ),
                        "db_path": LaunchConfiguration("db_path"),
                    },
                ],
            )
        ]
    )
