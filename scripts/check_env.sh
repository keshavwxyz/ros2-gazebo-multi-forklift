#!/usr/bin/env bash
# Make sure that the host has all the items that the forklift workspace uses.
# Exit code 0: all REQUIRED items are present. Exit code 1: a REQUIRED item is missing.
# Usage: scripts/check_env.sh
set -uo pipefail

if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi

missing_required=0
ok()   { printf '  \e[32mOK\e[0m    %s\n' "$*"; }
warn() { printf '  \e[33mWARN\e[0m  %s\n' "$*"; }
miss() { printf '  \e[31mMISS\e[0m  %s\n' "$*"; missing_required=1; }

echo "== Platform"
. /etc/os-release
[[ "$VERSION_ID" == "24.04" ]] && ok "Ubuntu $VERSION_ID" || warn "Ubuntu $VERSION_ID (24.04 expected)"
[[ "${ROS_DISTRO:-}" == "jazzy" ]] && ok "ROS 2 $ROS_DISTRO" || miss "ROS 2 Jazzy not sourced/installed"
if command -v gz >/dev/null; then
  gzv="$(gz sim --versions 2>/dev/null | head -1)"
  [[ "$gzv" == 8.* ]] && ok "Gazebo Sim $gzv (Harmonic)" || warn "Gazebo Sim $gzv (Harmonic 8.x expected for Jazzy)"
else
  miss "gz CLI (ros-jazzy-ros-gz)"
fi

echo "== ROS packages (required)"
pkgs="$(ros2 pkg list 2>/dev/null)"
required=(
  xacro urdf robot_state_publisher joint_state_publisher rviz2
  ros_gz_sim ros_gz_bridge ros_gz_image gz_ros2_control
  controller_manager diff_drive_controller joint_state_broadcaster
  position_controllers forward_command_controller imu_sensor_broadcaster
  nav2_bringup nav2_amcl nav2_map_server nav2_bt_navigator nav2_smac_planner
  nav2_regulated_pure_pursuit_controller nav2_velocity_smoother nav2_collision_monitor
  nav2_simple_commander slam_toolbox robot_localization twist_mux
  cv_bridge image_transport tf2_ros tf2_tools
  launch_testing launch_testing_ament_cmake ament_cmake_pytest
)
for p in "${required[@]}"; do
  grep -qx "$p" <<<"$pkgs" && ok "$p" || miss "$p  (apt: ros-jazzy-${p//_/-})"
done

echo "== ROS packages (optional)"
for p in tricycle_controller ackermann_steering_controller laser_filters rqt_tf_tree; do
  grep -qx "$p" <<<"$pkgs" && ok "$p" || warn "$p not installed (optional)"
done

echo "== Python"
python3 - <<'PY' || missing_required=1
import importlib, sys
bad = False
for name in ("numpy", "yaml", "pytest", "cv2"):
    try:
        m = importlib.import_module(name)
        print(f"  \033[32mOK\033[0m    {name} {getattr(m, '__version__', '')}")
    except ImportError:
        print(f"  \033[31mMISS\033[0m  {name}"); bad = True
try:
    import cv2
    assert hasattr(cv2, "QRCodeEncoder") and hasattr(cv2, "QRCodeDetector")
    img = cv2.QRCodeEncoder.create().encode("BOX_A:SHELF_01")
    img = cv2.copyMakeBorder(cv2.resize(img, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST),
                             32, 32, 32, 32, cv2.BORDER_CONSTANT, value=255)
    txt, _, _ = cv2.QRCodeDetector().detectAndDecode(img)
    assert txt == "BOX_A:SHELF_01", txt
    print("  \033[32mOK\033[0m    OpenCV QR encode/decode round trip")
except Exception as e:  # noqa: BLE001
    print(f"  \033[31mMISS\033[0m  OpenCV QR encode/decode ({e})"); bad = True
sys.exit(1 if bad else 0)
PY

echo "== GPU (Gazebo camera rendering)"
if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  ok "$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader)"
else
  warn "No NVIDIA GPU detected; Gazebo cameras will use software rendering (slow)"
fi

echo
if [[ $missing_required -eq 0 ]]; then echo "All required dependencies present."; else echo "Missing required dependencies (see MISS lines)."; fi
exit $missing_required
