"""demo.launch.py
────────────────
voice→DB Gate→motion→PLC LED 데모 통합 런치.

실행 예:
  # virtual (에뮬레이터):
  ros2 launch demo demo.launch.py

  # real (실물 110.120.1.38):
  ros2 launch demo demo.launch.py mode:=real robot_ip:=110.120.1.38

  # voice 포함:
  ros2 launch demo demo.launch.py mode:=real robot_ip:=110.120.1.38 voice:=true

  # 전체:
  ros2 launch demo demo.launch.py mode:=real robot_ip:=110.120.1.38 \\
    voice:=true plc:=true plc_port:=/dev/ttyUSB0 dashboard:=true

시작 순서 (타이머 활용):
  0s  — Doosan bringup (DSR + gripper_node)
  5s  — db_service_node
  7s  — orchestrator_node, tool_action_server, [plc_node]
  9s  — [whisper_node, rule_intent_node]  (voice:=true)
  10s — [dashboard_node]                  (dashboard:=true)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── Launch 인자 ──────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "mode", default_value="virtual",
            description="virtual | real"
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="110.120.1.38",
            description="Doosan 컨트롤러 IP (real 모드에서 사용)"
        ),
        DeclareLaunchArgument(
            "robot_ns", default_value="dsr01",
            description="Doosan 로봇 네임스페이스"
        ),
        DeclareLaunchArgument(
            "voice", default_value="false",
            description="음성 노드 활성화 (whisper + rule_intent)"
        ),
        DeclareLaunchArgument(
            "plc", default_value="false",
            description="PLC 노드 활성화"
        ),
        DeclareLaunchArgument(
            "plc_port", default_value="/dev/ttyUSB0",
            description="PLC 시리얼 포트"
        ),
        DeclareLaunchArgument(
            "dashboard", default_value="true",
            description="대시보드 노드 활성화 (http://localhost:8080)"
        ),
        DeclareLaunchArgument(
            "db_path", default_value="~/robot_tools.db",
            description="SQLite DB 경로"
        ),
    ]

    mode       = LaunchConfiguration("mode")
    robot_ip   = LaunchConfiguration("robot_ip")
    robot_ns   = LaunchConfiguration("robot_ns")
    voice      = LaunchConfiguration("voice")
    plc        = LaunchConfiguration("plc")
    plc_port   = LaunchConfiguration("plc_port")
    dashboard  = LaunchConfiguration("dashboard")
    db_path    = LaunchConfiguration("db_path")

    # ── 0s: Doosan bringup (DSR + gripper) ──────────────────────────────────
    # virtual 모드에서는 에뮬레이터가 127.0.0.1:12345에서 실행되므로 host를 고정한다.
    # real 모드에서는 사용자가 지정한 robot_ip를 그대로 사용한다.
    from launch.substitutions import PythonExpression
    bringup_host = PythonExpression(
        ["'127.0.0.1' if '", mode, "' == 'virtual' else '", robot_ip, "'"]
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("motion"),
                "launch",
                "bringup_e0509_with_gripper.launch.py",
            ])
        ]),
        launch_arguments={
            "mode":      mode,
            "host":      bringup_host,
            "robot_ip":  robot_ip,
            "name":      robot_ns,
        }.items(),
    )

    # ── 5s: DB 서비스 노드 ───────────────────────────────────────────────────
    db_node = Node(
        package="db",
        executable="db_service_node",
        name="db_service_node",
        output="screen",
        parameters=[{"db_path": db_path}],
    )

    # ── 7s: orchestrator + tool_action_server ───────────────────────────────
    orchestrator_node = Node(
        package="orchestrator",
        executable="orchestrator_node",
        name="orchestrator_node",
        output="screen",
        parameters=[{"robot_ns": robot_ns}],
    )

    tool_action_server = Node(
        package="motion",
        executable="tool_action_server",
        name="tool_action_server",
        output="screen",
        parameters=[{"robot_ns": robot_ns}],
    )

    # ── 7s: PLC 노드 (조건부) ───────────────────────────────────────────────
    plc_node = Node(
        package="plc",
        executable="plc_node",
        name="plc_node",
        output="screen",
        parameters=[{"port": plc_port}],
        condition=IfCondition(plc),
    )

    # ── 9s: 음성 노드 (조건부) ──────────────────────────────────────────────
    whisper_node = Node(
        package="voice",
        executable="whisper_node",
        name="whisper_node",
        output="screen",
        condition=IfCondition(voice),
    )
    rule_intent_node = Node(
        package="voice",
        executable="rule_intent_node",
        name="rule_intent_node",
        output="screen",
        condition=IfCondition(voice),
    )

    # ── 10s: 대시보드 (조건부) ──────────────────────────────────────────────
    dashboard_node = Node(
        package="dashboard",
        executable="dashboard_node",
        name="dashboard_node",
        output="screen",
        parameters=[{
            "db_path":     db_path,
            "gripper_cam": "/dev/gripper_cam",
            "top_cam":     "/dev/top_cam",
        }],
        condition=IfCondition(dashboard),
    )

    return LaunchDescription(args + [
        # t=0: Doosan bringup
        bringup,

        # t=5: DB 서비스 (로봇 컨트롤러 연결 후)
        TimerAction(period=5.0, actions=[db_node]),

        # t=7: orchestrator + motion 액션서버 + PLC
        TimerAction(period=7.0, actions=[
            orchestrator_node,
            tool_action_server,
            plc_node,
        ]),

        # t=9: 음성 노드
        TimerAction(period=9.0, actions=[
            whisper_node,
            rule_intent_node,
        ]),

        # t=10: 대시보드
        TimerAction(period=10.0, actions=[dashboard_node]),
    ])
