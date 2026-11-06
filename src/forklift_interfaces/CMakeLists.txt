cmake_minimum_required(VERSION 3.8)
project(forklift_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QrDetection.msg"
  "msg/QrDetectionArray.msg"
  "msg/ForkState.msg"
  "msg/MissionStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/RobotStatus.msg"
  "msg/TaskState.msg"
  "msg/Reservation.msg"
  "msg/ConflictEvent.msg"
  "msg/FleetStatus.msg"
  "srv/ClaimTask.srv"
  "action/SetForkHeight.action"
  "action/RunMission.action"
  "action/PickupLoad.action"
  "action/PlaceLoad.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
