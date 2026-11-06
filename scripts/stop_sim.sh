#!/usr/bin/env bash
# Stop all processes of a forklift simulation/bringup (Gazebo, bridge, ROS nodes).
# Use this script after an interrupted launch. Remaining processes can send duplicate data.
patterns=(
  'ros2 launch forklift_'
  'gz sim'
  'ros_gz_bridge/parameter_bridge'
  'ros_gz_sim/create'
  'robot_state_publisher/robot_state_publisher'
  'controller_manager/ros2_control_node'
  'controller_manager/spawner'
  'twist_mux/twist_mux'
  'forklift_control/lift_controller'
  'robot_localization/ekf_node'
  'rviz2/rviz2'
)
for p in "${patterns[@]}"; do pkill -INT -f "$p" 2>/dev/null; done
for _ in $(seq 1 50); do
  alive=0
  for p in "${patterns[@]}"; do pgrep -f "$p" >/dev/null && alive=1; done
  [[ $alive -eq 0 ]] && { echo "simulation stopped"; exit 0; }
  sleep 0.2
done
for p in "${patterns[@]}"; do pkill -KILL -f "$p" 2>/dev/null; done
echo "simulation killed (did not exit on SIGINT)"
