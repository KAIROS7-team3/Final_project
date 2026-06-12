# e0509 + RH-P12-RN 그리퍼 통합 bringup (실물 로봇 전용)
#
# 실행 예:
#   ros2 launch motion bringup_e0509_with_gripper.launch.py \
#     host:=110.120.1.38 robot_ip:=110.120.1.38

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from dsr_bringup2.utils import read_update_rate


def generate_launch_description() -> LaunchDescription:
    pkg_motion = get_package_share_directory("motion")
    gripper_urdf_path = os.path.join(pkg_motion, "urdf", "e0509_with_gripper.urdf")
    gripper_config = os.path.join(pkg_motion, "config", "gripper_node.yaml")

    with open(gripper_urdf_path, encoding="utf-8") as f:
        gripper_urdf = f.read()

    update_rate = str(read_update_rate())

    arguments = [
        DeclareLaunchArgument("name",     default_value="dsr01",          description="Robot namespace"),
        DeclareLaunchArgument("host",     default_value="110.120.1.38",   description="Doosan controller IP"),
        DeclareLaunchArgument("port",     default_value="12345",           description="Doosan controller port"),
        DeclareLaunchArgument("model",    default_value="e0509",           description="Robot model"),
        DeclareLaunchArgument("color",    default_value="white",           description="Mesh color"),
        DeclareLaunchArgument("rt_host",  default_value="192.168.137.50", description="RT IP"),
        DeclareLaunchArgument("robot_ip", default_value=LaunchConfiguration("host"),
                              description="Gripper TCP IP"),
        DeclareLaunchArgument("launch_gripper", default_value="true",
                              description="gripper_node 실행"),
        DeclareLaunchArgument("launch_merger", default_value="true",
                              description="rviz_joint_state_merger 실행"),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration("model")]),
            ".urdf.xacro",
            " name:=", LaunchConfiguration("name"),
            " host:=", LaunchConfiguration("host"),
            " rt_host:=", LaunchConfiguration("rt_host"),
            " port:=", LaunchConfiguration("port"),
            " mode:=real",
            " model:=", LaunchConfiguration("model"),
            " update_rate:=", update_rate,
        ]
    )

    robot_controllers = [
        PathJoinSubstitution([FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]),
    ]

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        parameters=[{"robot_description": robot_description_content}] + robot_controllers,
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        remappings=[("joint_states", "joint_states_rviz")],
        parameters=[{"robot_description": gripper_urdf}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    gripper_service_node = Node(
        package="motion",
        executable="gripper_node",
        name="gripper_node",
        output="screen",
        parameters=[
            gripper_config,
            {"robot_ip": LaunchConfiguration("robot_ip")},
            {"mode": "real"},
        ],
        condition=IfCondition(LaunchConfiguration("launch_gripper")),
    )

    joint_state_merger_node = Node(
        package="motion",
        executable="rviz_joint_state_merger",
        name="rviz_joint_state_merger",
        output="screen",
        parameters=[{"robot_ns": LaunchConfiguration("name")}],
        condition=IfCondition(LaunchConfiguration("launch_merger")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("name"),
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
        )],
    )

    # real 전용: TCP/Tool 설정만 수행 (홈 이동은 스킵 — 현재 자세 불명으로 충돌 위험)
    home_on_start_node = Node(
        package="motion",
        executable="home_on_start",
        name="home_on_start",
        output="screen",
        parameters=[
            {"robot_ns": LaunchConfiguration("name")},
            {"mode": "real"},
        ],
    )

    delay_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                gripper_service_node,
                joint_state_merger_node,
                rviz_node,
                # real: TCP 설정만 (S-5 — 이전 데모 실패 원인인 TCP Z+160 미적용 방지)
                TimerAction(period=3.0, actions=[home_on_start_node]),
            ],
        )
    )

    return LaunchDescription(
        arguments + [
            robot_state_pub_node,
            control_node,
            robot_controller_spawner,
            joint_state_broadcaster_spawner,
            delay_after_controller,
        ]
    )
