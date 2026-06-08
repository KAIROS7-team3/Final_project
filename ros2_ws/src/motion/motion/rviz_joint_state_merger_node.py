"""Merge Doosan arm /joint_states with RH-P12-RN gripper pulse feedback for RViz."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from motion.gripper_conversion import pulse_to_rad

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINTS = ["rh_r1", "rh_l1", "rh_r2", "rh_l2"]
GRIPPER_FEEDBACK_NAME = "gripper_joint"


class RvizJointStateMerger(Node):
    def __init__(self) -> None:
        super().__init__("rviz_joint_state_merger")

        self.declare_parameter("robot_ns", "dsr01")
        self.declare_parameter("arm_joint_states_topic", "")
        self.declare_parameter("gripper_state_topic", "/gripper/state")
        self.declare_parameter("output_joint_states_topic", "")
        self.declare_parameter("open_rad", 0.0)
        self.declare_parameter("closed_rad", 1.101)
        self.declare_parameter("pulse_open", 0)
        self.declare_parameter("pulse_closed", 700)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("publish_all_gripper_joints", True)
        self.declare_parameter("poll_gripper_service", False)
        self.declare_parameter("gripper_service_name", "/gripper/get_state")

        robot_ns = self.get_parameter("robot_ns").get_parameter_value().string_value
        arm_topic = self.get_parameter("arm_joint_states_topic").get_parameter_value().string_value
        if not arm_topic:
            arm_topic = f"/{robot_ns}/joint_states"
        gripper_topic = self.get_parameter("gripper_state_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_joint_states_topic").get_parameter_value().string_value
        if not out_topic:
            out_topic = f"/{robot_ns}/joint_states_rviz"

        self._open_rad = self.get_parameter("open_rad").value
        self._closed_rad = self.get_parameter("closed_rad").value
        self._pulse_open = int(self.get_parameter("pulse_open").value)
        self._pulse_closed = int(self.get_parameter("pulse_closed").value)
        self._publish_all_gripper = self.get_parameter("publish_all_gripper_joints").value

        self._arm_msg: JointState | None = None
        self._gripper_pulse = 0.0
        self._have_gripper = False

        arm_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, arm_topic, self._on_arm, arm_qos)
        gripper_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, gripper_topic, self._on_gripper, gripper_qos)
        self._pub = self.create_publisher(JointState, out_topic, 10)

        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._publish)
        self.create_timer(5.0, self._log_status)

        self.get_logger().info(f"[merger] arm={arm_topic} gripper={gripper_topic} -> {out_topic}")

    def _on_arm(self, msg: JointState) -> None:
        self._arm_msg = msg

    def _on_gripper(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == GRIPPER_FEEDBACK_NAME:
                self._gripper_pulse = pos
                self._have_gripper = True
                return

    def _log_status(self) -> None:
        if self._arm_msg is None:
            self.get_logger().warn("[merger] 팔 joint_states 미수신 — dsr_controller2 확인")
            return
        if not self._have_gripper:
            self.get_logger().warn(
                "[merger] 그리퍼 pulse 미수신 — gripper_node TCP/DRL 로그 확인"
            )
        else:
            self.get_logger().info(
                f"[merger] gripper pulse={self._gripper_pulse:.0f} -> rh_r1={self._master_rad():.3f} rad",
                throttle_duration_sec=5.0,
            )

    def _master_rad(self) -> float:
        return pulse_to_rad(
            int(round(self._gripper_pulse)),
            open_rad=self._open_rad,
            closed_rad=self._closed_rad,
            pulse_open=self._pulse_open,
            pulse_closed=self._pulse_closed,
        )

    def _publish(self) -> None:
        if self._arm_msg is None:
            return

        src = self._arm_msg
        names = list(src.name)
        positions = list(src.position)
        velocities = list(src.velocity) if src.velocity else [0.0] * len(names)
        efforts = list(src.effort) if src.effort else [0.0] * len(names)

        while len(velocities) < len(names):
            velocities.append(0.0)
        while len(efforts) < len(names):
            efforts.append(0.0)

        master = self._master_rad() if self._have_gripper else 0.0
        gripper_targets = GRIPPER_JOINTS if self._publish_all_gripper else [GRIPPER_JOINTS[0]]
        for gj in gripper_targets:
            if gj in names:
                positions[names.index(gj)] = master
            else:
                names.append(gj)
                positions.append(master)
                velocities.append(0.0)
                efforts.append(0.0)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = positions
        out.velocity = velocities
        out.effort = efforts
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizJointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
