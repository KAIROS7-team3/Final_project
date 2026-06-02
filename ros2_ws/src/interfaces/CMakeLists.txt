cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/ToolStatus.msg"
  "msg/PLCStatus.msg"
  "msg/RobotStatus.msg"
  "msg/Intent.msg"
  "msg/MarkerMap.msg"
  # Services
  "srv/CheckToolFeasibility.srv"
  "srv/UpdateToolStatus.srv"
  # Actions
  "action/MoveToPose.action"
  "action/Grasp.action"
  "action/Release.action"
  "action/PlaceAtStaging.action"
  "action/PickFromStaging.action"
  "action/ReturnToSlot.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
