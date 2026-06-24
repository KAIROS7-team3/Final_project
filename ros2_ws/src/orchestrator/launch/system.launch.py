"""system.launch.py
──────────────────
Track A 전체 시스템 런치 (프로덕션).

voice→DB Gate→motion→PLC LED 전체 스택을 기동한다.
demo_trigger 같은 수동 주입 도구는 포함하지 않는다 — 입력은 whisper_node 음성 파이프라인.

실행 예:
  # virtual (에뮬레이터, 음성 없이 기본 기동):
  ros2 launch orchestrator system.launch.py

  # real (실물, PLC USB 연결 시 /dev/plc 심링크 자동 사용):
  ros2 launch orchestrator system.launch.py mode:=real robot_ip:=110.120.1.38

  # 음성 포함:
  ros2 launch orchestrator system.launch.py mode:=real robot_ip:=110.120.1.38 voice:=true

  # PLC 비활성화 (PLC 미연결 시):
  ros2 launch orchestrator system.launch.py mode:=real robot_ip:=110.120.1.38 plc:=false

  # 전체:
  ros2 launch orchestrator system.launch.py mode:=real robot_ip:=110.120.1.38 \\
    voice:=true dashboard:=true

시작 순서:
  0s  — Doosan bringup (DSR + gripper_node)
  5s  — db_service_node
  7s  — orchestrator_node, tool_action_server, [plc_node]
  9s  — [whisper_node, rule_intent_node]  (voice:=true)
  10s — [dashboard_node]                  (dashboard:=true)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
            "plc", default_value="true",
            description="PLC 노드 활성화 (PLC 미연결 시 false)"
        ),
        DeclareLaunchArgument(
            "plc_port", default_value="/dev/plc",
            description="PLC 시리얼 포트 (udev 심링크 — scripts/udev/99-robot.rules)"
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

    mode      = LaunchConfiguration("mode")
    robot_ip  = LaunchConfiguration("robot_ip")
    robot_ns  = LaunchConfiguration("robot_ns")
    voice     = LaunchConfiguration("voice")
    plc       = LaunchConfiguration("plc")
    plc_port  = LaunchConfiguration("plc_port")
    dashboard = LaunchConfiguration("dashboard")
    db_path   = LaunchConfiguration("db_path")

    # ── 0s: Doosan bringup (DSR + gripper) ──────────────────────────────────
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
            "mode":     mode,
            "host":     bringup_host,
            "robot_ip": robot_ip,
            "name":     robot_ns,
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

    # ── 7s: orchestrator + tool_action_server + PLC ─────────────────────────
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

    plc_node = Node(
        package="plc",
        executable="plc_node",
        name="plc_node",
        output="screen",
        parameters=[{"port": plc_port}],
        condition=IfCondition(plc),
    )

    # ── 9s: 음성 파이프라인 (조건부) ────────────────────────────────────────
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
            "gripper_cam": "/dev/cam_wrist",
            "top_cam":     "/dev/video6",
        }],
        condition=IfCondition(dashboard),
    )

    return LaunchDescription(args + [
        bringup,
        TimerAction(period=5.0,  actions=[db_node]),
        TimerAction(period=7.0,  actions=[orchestrator_node, tool_action_server, plc_node]),
        TimerAction(period=9.0,  actions=[whisper_node, rule_intent_node]),
        TimerAction(period=10.0, actions=[dashboard_node]),
    ])
