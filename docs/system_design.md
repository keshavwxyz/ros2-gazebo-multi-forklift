# System Design and Decision Record

Status legend: **Decided** = the implementation uses this decision; **Provisional** = default
selection, examine it again after the named phase validates it.

The decisions were made for this environment (examined 2026-10-28, refer to
`scripts/check_env.sh`): Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Sim 8.15 (Harmonic, ROS vendor
packages), gz_ros2_control 1.2.20, ros2_control 4.48, Nav2 1.3.13, slam_toolbox 2.8.5,
robot_localization 3.8.3, OpenCV 4.6.0, RTX 5060 Laptop (8 GB), 16 GB RAM.

---

## D1. Simulator: Gazebo Sim Harmonic through `ros_gz` — Decided
Harmonic is the official Gazebo version for Jazzy. The `gz_*_vendor` packages install it.
Gazebo Classic does not operate on 24.04/Jazzy, so it is not an option. Physics engine: DART
(the Gazebo default). A headless run (`gz sim -s -r`) was verified on the test computer.

## D2. Drive kinematics: differential drive — Decided (validated in Gazebo)
| Option | Realism | Nav2 support | Precision docking | Risk |
|---|---|---|---|---|
| Rear-axle Ackermann (counterbalance forklift) | High | Smac Hybrid-A*/Lattice + RPP/MPPI-Ackermann | Difficult, no turn in position | High |
| Tricycle: steered rear drive wheel (AGV pallet stacker) | High for AGVs | Similar to diff drive if the steering gets to ±90° | Good, but steering lag | Medium |
| **Differential: 2 driven front wheels + 2 rear casters** | Medium (real AMR stackers are available) | Native | Best: turns in position around the drive axle near the forks | **Low** |

Selection: differential drive. The drive axle is below the mast, so the robot turns near the
forks. This makes the last fork alignment simple and robust. The model has **4 wheels**:
2 driven wheels and 2 rear casters. To prevent swivel-joint jitter in DART, the casters are
spheres with no friction (an approximation, refer to [robot_model.md](robot_model.md)).
Upgrade path: `tricycle_controller` (installed) uses the same `TwistStamped` `cmd_vel` input.
Thus a steered drive wheel would change the URDF and the controller YAML, but not the autonomy
code. Nav2 would then need a different `motion_model` and new controller tuning.

Finding: with cylinder wheel collisions, DART put the contacts at the outer edge of the rim
(effective track 0.92 m, not 0.80 m). This gave a 13% yaw-odometry error. Now the drive wheels
have sphere collisions, and the odometry agrees with ground truth in 0.1°
([sensor_setup.md](sensor_setup.md)).

## D3. Control: ros2_control with replaceable hardware backends — Decided
One URDF `<ros2_control>` block. The xacro argument `sim_backend` selects the plugin:

| `sim_backend` | Plugin | Use |
|---|---|---|
| `gazebo` | `gz_ros2_control/GazeboSimSystem` | Gazebo Harmonic |
| `mock` | `mock_components/GenericSystem` | CI and integration tests with **no simulator** |

Controllers: `diff_drive_controller` (drive, `enable_odom_tf: false`), a
`joint_trajectory_controller` on the **velocity** interface of the lift joint (refer to D4),
and `joint_state_broadcaster`. The IMU is a Gazebo sensor, and the bridge sends it to ROS. It
does not go through `imu_sensor_broadcaster`. Thus an adapter for a different simulator can
replace it with one IMU publisher.

## D4. Lift mechanism — Decided (validated in Gazebo)
The mast is fixed to the chassis. `fork_lift_joint` is a **prismatic** carriage joint with hard
limits, velocity limits and effort limits. The forks are rigidly attached to the carriage.
`forklift_control/lift_controller` gives the `SetForkHeight` action. The controller:
* refuses goals out of the limits (it does not clamp them);
* plans a trapezoidal profile (≤0.15 m/s, 0.3 m/s²) in the joint velocity limit;
* sends the profile to the trajectory controller;
* monitors the motion (tracking error against the profile *at the time stamp of the
  measurement*, timeout, old joint states);
* publishes `ForkState`.

**Velocity command interface, not position.** This is similar to a hydraulic valve that
controls flow, and the simulator also makes it necessary. gz_ros2_control changes a position
command into a velocity request of gain × error × rate. In DART, a request above the velocity
limit of the joint **freezes the joint. It does not limit the velocity.** Measured: a request
of 0.15 m/s moved the joint, and 1.5 m/s froze it. Thus all useful position gains fail on step
commands. The trajectory controller on the velocity interface follows the bounded profile
(feed-forward and P). It holds the last point itself, so the forks stay in position if the
monitor node stops.

**Monitor that does not block.** The first version used a loop in the execute callback of the
action. With the rclpy multi-threaded executor, this stopped the subscriptions of the node for
up to ~0.9 s (incorrect "stale joint states" and tracking faults). Now the monitor operates in
a timer, and the execute coroutine waits on a future, all on a single-threaded executor.
Mast tilt is not included (it is optional in the specification, and it adds risk to the load
stability).

**Load sensing:** the lift joint effort (gz_ros2_control effort state interface) increases with
the supported mass. With empty forks, it is 1079.1 N = 110 kg × g in Gazebo. This confirms
that the signal is physical. It is the simulated equivalent of the hydraulic-pressure load
sensor of a forklift. The system uses it to *verify* the pickup and the placement without
simulator ground truth.

## D5. Load handling: physical contact — Decided (validated in Gazebo; [load_handling.md](load_handling.md))
1. **Primary:** each box is one rigid model: a pallet base with two fork pockets and the box
   body. The forks go into the pockets and lift by contact. Friction and low acceleration
   limits keep the load on the forks. No attachment, no teleport.
2. **Fallback** (only if contact slip was not reliable): the Gazebo `DetachableJoint` system
   makes or removes a fixed joint between the carriage and the pallet. It is permitted only
   when *all* these conditions are true: the forks are inserted (alignment in tolerance), the
   fork height is in the pickup band, and the lift effort is above the load threshold. It is
   released only after the effort decreases at the destination. The joint is made at the
   current relative pose, so nothing teleports.

Result: the primary method operates correctly. With the forks in the pockets and lifted
0.25 m, the pallet stayed on the forks during a full Nav2 transport (drift ≤ 1.5 cm, constant
height, zero tilt). Thus the attachment fallback was not implemented. A static test prevents
attachment joints and entity-pose services in the autonomy code. The lift effort verifies the
pickup and the placement (30.0 kg measured for a 30 kg load). The camera also verifies the
placement.

## D6. Sensors and mount positions — Decided
| Sensor | Mount (in `base_footprint`) | Default configuration | Reason |
|---|---|---|---|
| 2D LiDAR | Front center of the chassis, x=0.25, z=0.12, between the mast uprights | ~190° FOV (±95°), 10 Hz, 0.1–12 m | The scan plane goes below the lowest carriage bar (0.26 m), below the mast (starts at 0.17 m) and above lowered tines (0.05 m). It sees below a load that is lifted more than ~0.1 m. **Self-occlusion:** when the forks are below ~0.11 m, the fork shanks cross the plane at bearings ±61°. A scan filter masks those sectors. Rays out of ~±99° would hit the front face of the chassis, so the FOV is limited. A rear LiDAR covers the rear (refer to [multi_robot_coordination.md](multi_robot_coordination.md#rear-lidar)). |
| RGB camera | Below the front edge of the overhead guard, x=0.14, z=2.00, 40° down | 1280×720, 70° HFOV, 10 Hz | The first plan was on the mast crossbar at ≈1.0 m. The geometry showed that the moving carriage would block that view during the lift. Thus the camera moved above the highest carriage position. Its frame does not change with the lift state. At 40° down, the fork tips are at the bottom of the image, and a QR code at 0.4 m height is visible from ≈1.0 m to ≈4.9 m. Tests make sure of the two conditions. |
| IMU | Center of the chassis, x=−0.45, z=0.30 | 100 Hz | Yaw rate for the EKF |
| Wheel encoders | Drive joints through ros2_control | controller rate 50 Hz | `diff_drive_controller` odometry |
| Fork height | Lift joint state | 50 Hz | Already in `/joint_states` |

QR size: the payload `BOX_A:SHELF_01` (14 bytes) fits in QR version 1 (21×21 modules). At a
printed size of 0.30 m and a range of 2.5 m, the camera above gives ≈3 px/module before
foreshortening (the lifted camera sees the QR code ≈30–40° off-axis). This is near the
practical decode limit. [perception.md](perception.md) gives the measured decode range.

## D7. State estimation and TF authority — Decided
* `odom → base_footprint`: **robot_localization EKF** (`two_d_mode`). It uses only the
  wheel-odometry *velocities* (vx, vyaw) and the IMU *yaw rate*. It does not use:
  * the odometry pose (this would count the same encoder data two times);
  * the IMU absolute orientation (the Gazebo IMU orientation is world-referenced, and thus
    almost ground truth);
  * the IMU acceleration (noisy double integration adds little at these speeds).
* `smooth_lagged_data: true` (1 s history): the wheel odometry and the IMU come from different
  processes, and sometimes the odometry arrives ~350 ms late in a burst. Without smoothing, the
  EKF dropped those measurements, and one time in the Gazebo suite it estimated a 2 m motion as
  0.25 m too short. A deterministic regression test
  (`forklift_localization/test/test_ekf_lagged_odometry.py`) shows the problem: 0.85–0.90 m,
  not 1.00 m, without smoothing, and the correct value with it. The same test then found a
  second cause: the default odometry queue (10) overflowed on a burst of ~17–20 late messages
  and dropped the oldest messages without a warning. Now the queues hold 1 s or more (odom 50,
  IMU 100).
* The acceleration states get ≈ 0 process noise (no sensor measures them). With the default
  values, the odom position of the EKF sometimes moved 0.3–0.5 m to the rear at the first
  motion (6 of 10 runs), and AMCL then showed a localization spike (0 of 6 after the solution;
  refer to [navigation.md](navigation.md)).
* `map → odom`: **slam_toolbox** in mapping mode, **AMCL** in navigation mode. Never the two
  at the same time.
* All the frames below `base_footprint`: `robot_state_publisher` from the URDF and
  `/joint_states`.
* Gazebo publishes **no TF to ROS**. The bridge sends ground truth only to `/ground_truth/odom`,
  and only the evaluation tests use it.

## D8. Navigation — Decided (validated in Gazebo; refer to [navigation.md](navigation.md))
* Planner: Smac 2D (or NavFn). Controller: Regulated Pure Pursuit (deterministic, low cost,
  rotate-to-heading for a differential drive). MPPI is the fallback if RPP has problems in
  narrow aisles.
* Velocity smoother → collision monitor → command arbiter.
* Footprint: a polygon that covers the chassis **and the forks**. With a load, the task manager
  publishes a larger footprint on the `footprint` topic of the costmaps (Nav2 accepts footprint
  changes during operation).
* **Nav2 does not do the last fork insertion.** The pallet is an obstacle in the costmap. Nav2
  moves the robot to a *staging pose* approximately 1.5 m in front of the target. A dedicated
  camera-guided approach controller then aligns and inserts at low speed. It refuses to insert
  when the lateral or heading error is more than the tolerance.
* In Jazzy, `diff_drive_controller` uses `geometry_msgs/TwistStamped`. The Nav2
  `enable_stamped_cmd_vel` is false by default in Jazzy. Thus it is set to true on each
  velocity publisher (tested).
* Result: Smac 2D planning on a 0.66 m circle (the global costmap) and RPP with polygon
  footprints that include the forks (local costmap). A scan self-filter removes the fork-shank
  returns. The footprint covers the drive-wheel collision spheres (±0.61 m). A URDF-based test
  found this problem.

## D9. Command arbitration and safety — Decided (validated in Gazebo)
`twist_mux` (installed) is in front of the drive controller: e-stop lock > teleop > approach
controller > Nav2, with a timeout for each source. Thus a publisher that stops causes the robot
to stop. Inputs: `/cmd_vel/teleop`, `/cmd_vel/approach`, `/cmd_vel/nav` →
`/forklift_base_controller/cmd_vel`. `twist_mux` accepts `use_stamped: true` (verified), so no
relay is necessary. Tests make sure that the e-stop lock (`/estop`, priority 255, no timeout)
stops the motion, that its release starts the motion again, and that the robot brakes when the
commands stop (mux timeout and `cmd_vel_timeout` dead-man).

## D10. Perception — Decided (validated in Gazebo)
Only OpenCV: `cv2.QRCodeDetector` to decode, `cv2.QRCodeEncoder` to make the assets. A round
trip was verified on the test computer, also with perspective warp and blur. This prevents
`pyzbar`/`qrcode` dependencies. pyzbar is an apt-installable fallback if the OpenCV detector
is weak on rendered images. The relative pose comes from `solvePnP` (`SOLVEPNP_IPPE_SQUARE`)
with the 4 QR corners, the known printed size and `camera_info`. The node compares the payloads
with the single task configuration. It refuses unknown or malformed payloads and gives a
reason. The perception node does not read simulator entity names.

Result ([perception.md](perception.md)): the combined multi-code decoder of OpenCV attached one
payload to two symbols. It was replaced by "locate all, then decode each crop separately", with
a perspective-rectified fallback for steep views at short range. Decoding is 100% from 4.25 m
to 2.25 m. The world-pose error is 1–13 mm and ≤1.1° in Gazebo. The mission configuration moved
into `forklift_task_manager` (single source of truth). The perception package imports its
validator, so there is one definition of a valid payload.

## D11. Task orchestration: explicit state machine in Python — Decided
An explicit state machine (a table of state → handler, with a timeout and a failure transition
for each state), not py_trees:
* py_trees is not installed;
* the mission is a linear sequence with recovery branches;
* a table-driven state machine is easier to unit-test and to explain.

The core state machine is pure Python with no rclpy, so the unit tests do not need ROS. A thin
ROS node operates it with timers and async action and service futures. The callbacks do not
block.

## D12. Configuration layout — Decided (different from the suggested tree)
The specification suggests a top-level `config/`, `maps/` and `worlds/`. ROS 2 launch files
find resources through `get_package_share_directory`, which sees only the files that a package
installs. Thus the configurations are in their **owner package** (`src/<pkg>/config/...`), the
maps in `forklift_navigation/maps`, and the worlds in `forklift_simulation/worlds`. The single
source of truth for the boxes and destinations is
`forklift_task_manager/config/warehouse_mission.yaml`. The files for a specified simulator
(worlds, bridge YAML, gz plugin xacro) are only in `forklift_simulation`.

## D13. Canonical topic contract (simulator adapter boundary) — Decided
The autonomy packages use only these names. Each simulator adapter (for example the Gazebo
bridge YAML) must supply them:
`/clock`, `/scan`, `/camera/image_raw`, `/camera/camera_info`, `/imu/data`, `/joint_states`
(through ros2_control), with `/forklift_base_controller/cmd_vel` and the lift controller
command as inputs.

## D14. Test tiers — Decided
1. **Unit** (pytest, no ROS graph): payload parsing, configuration validation, mission queue,
   state-machine transitions, fork limits.
2. **Integration without a simulator** (launch_testing + `sim_backend:=mock`): controllers,
   lift action, TF tree, perception on synthetic images.
3. **Simulation** (launch_testing + headless Gazebo): sensors, motion, QR decode from rendered
   images, pickup and placement, full mission.

If Gazebo is not available (for example in CI), the tier-3 tests are skipped and reported as
**not executed**.

## Known environment problem
On the network of the test computer, the IPv6 connection to `download.ros.org` times out.
libxml2 (which `ament_xmllint` uses) tries all the AAAA records first. Thus the package.xml lint
takes approximately 3 minutes and fails the 60 s timeout of colcon. `source scripts/env.sh`
installs an offline XML catalog to prevent this (the lint now takes 0.1 s). The system-wide
solution is to prefer IPv4 in `/etc/gai.conf`.

---

## Implementation plan
| Phase | Deliverable | Exit check |
|---|---|---|
| 0 ✅ | Environment audit, skeleton, architecture, interfaces | `check_env.sh` passes; `forklift_interfaces` builds and passes lint |
| 1 ✅ | `forklift_description` xacro (chassis, 4 wheels, mast, carriage, forks, sensor frames), parameter YAML | `xacro` → `check_urdf`; pytest on joint limits, inertias and frames; mock ros2_control launch test; RViz view |
| 2 ✅ | `forklift_simulation` world and spawn; `forklift_control` controllers, lift action, twist_mux | Mock-backend launch test; headless Gazebo: move and stop, lift to the limits |
| 3 ✅ | Sensors and bridge | Rate, type, frame and TF tests on headless Gazebo |
| 4 ✅ | `forklift_perception`, QR asset generator, textured boxes | Unit tests on synthetic images; decode from rendered Gazebo frames |
| 5 ✅ | `forklift_localization` (EKF), slam_toolbox mapping, saved map, AMCL, `forklift_navigation` | Map saved and loaded again; goal reached; EKF error against ground truth in the limit |
| 6 ✅ | Approach controller, pickup and placement | One box picked up, moved and placed in Gazebo |
| 7 ✅ | `forklift_task_manager` state machine and mission | The three-box mission completes headless |
| 8 ✅ | Failure-case tests, CI, documents | All the tiers operate; the items that are not tested are listed |
| 9 ✅ | `forklift_fleet`: two forklifts, fleet coordinator, rear LiDAR | The two-forklift mission completes 3/3 in Gazebo with no overlap |
