"""ROS2 의존성 모킹 — rclpy 없이 순수 Python 헬퍼 테스트용."""
import sys
import types
from unittest.mock import MagicMock

# ---------- rclpy stub ----------
rclpy_mod = types.ModuleType("rclpy")
rclpy_mod.init = MagicMock()
rclpy_mod.spin = MagicMock()
rclpy_mod.shutdown = MagicMock()
node_mod = types.ModuleType("rclpy.node")
node_mod.Node = object
qos_mod = types.ModuleType("rclpy.qos")
qos_mod.QoSProfile = MagicMock()
qos_mod.QoSReliabilityPolicy = MagicMock()
qos_mod.qos_profile_sensor_data = MagicMock()
rclpy_mod.node = node_mod
rclpy_mod.qos = qos_mod
sys.modules.setdefault("rclpy", rclpy_mod)
sys.modules.setdefault("rclpy.node", node_mod)
sys.modules.setdefault("rclpy.qos", qos_mod)

# ---------- rclpy.time stub ----------
time_mod = types.ModuleType("rclpy.time")
time_mod.Time = MagicMock()
rclpy_mod.time = time_mod
sys.modules.setdefault("rclpy.time", time_mod)

# ---------- geometry_msgs stub ----------
gm = types.ModuleType("geometry_msgs")
gm_msg = types.ModuleType("geometry_msgs.msg")
gm_msg.PointStamped = MagicMock()
gm_msg.TransformStamped = MagicMock()
gm.msg = gm_msg
sys.modules.setdefault("geometry_msgs", gm)
sys.modules.setdefault("geometry_msgs.msg", gm_msg)

# ---------- tf2_ros stub ----------
tf2 = types.ModuleType("tf2_ros")
tf2.Buffer = MagicMock()
tf2.TransformListener = MagicMock()


class _LookupException(Exception):
    pass


class _ConnectivityException(Exception):
    pass


class _ExtrapolationException(Exception):
    pass


tf2.LookupException = _LookupException
tf2.ConnectivityException = _ConnectivityException
tf2.ExtrapolationException = _ExtrapolationException
sys.modules.setdefault("tf2_ros", tf2)

# ---------- sensor_msgs stub ----------
sm = types.ModuleType("sensor_msgs")
sm_msg = types.ModuleType("sensor_msgs.msg")
sm_msg.Image = MagicMock()
sm.msg = sm_msg
sys.modules.setdefault("sensor_msgs", sm)
sys.modules.setdefault("sensor_msgs.msg", sm_msg)

# ---------- std_msgs stub ----------
std = types.ModuleType("std_msgs")
std_msg = types.ModuleType("std_msgs.msg")
std_msg.String = MagicMock()
std.msg = std_msg
sys.modules.setdefault("std_msgs", std)
sys.modules.setdefault("std_msgs.msg", std_msg)

# ---------- vision_msgs stub ----------
vm = types.ModuleType("vision_msgs")
vm_msg = types.ModuleType("vision_msgs.msg")
for _cls in (
    "BoundingBox2D", "BoundingBox3D",
    "Detection2D", "Detection2DArray",
    "Detection3D", "Detection3DArray",
    "ObjectHypothesisWithPose",
):
    setattr(vm_msg, _cls, MagicMock())
vm.msg = vm_msg
sys.modules.setdefault("vision_msgs", vm)
sys.modules.setdefault("vision_msgs.msg", vm_msg)

# ---------- cv_bridge stub ----------
cvb = types.ModuleType("cv_bridge")
cvb.CvBridge = MagicMock()
sys.modules.setdefault("cv_bridge", cvb)

# ---------- message_filters stub ----------
mf = types.ModuleType("message_filters")
mf.ApproximateTimeSynchronizer = MagicMock()
mf.Subscriber = MagicMock()
sys.modules.setdefault("message_filters", mf)

# ---------- cv2 / ultralytics stub ----------
sys.modules.setdefault("cv2", MagicMock())
sys.modules.setdefault("ultralytics", MagicMock())
