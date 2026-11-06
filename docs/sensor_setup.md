# Sensors, Topics and Odometry

Status: **complete and verified in Gazebo**. `forklift_test/test/test_sim_sensors_tf.py`
checked all the values below on the headless simulation. There are no placeholder values. All
the data comes from Gazebo sensor models or from ros2_control.

## Sensor specifications
`forklift_description/config/forklift_params.yaml` (`sensors:`) defines the sensors one time,
independently of the simulator. `forklift_simulation/urdf/forklift_gazebo.xacro` makes Gazebo
sensors from these specifications. An adapter for a different simulator can use the same
specifications.

| Sensor | Gazebo type | Rate | Main parameters | Noise model |
|---|---|---|---|---|
| 2D LiDAR (front) | `gpu_lidar` | 10 Hz | 381 beams in ±95° (0.5°), 0.10–12 m | Gaussian σ = 1 cm |
| 2D LiDAR (rear) | `gpu_lidar` | 10 Hz | 181 beams in ±90° (1°), 0.10–8 m, points to the rear | Gaussian σ = 1 cm |
| RGB camera | `camera` | 10 Hz | 1280×720, 70° HFOV, clip 0.05–30 m, 40° down | Gaussian σ = 0.005 (normalized intensity) |
| IMU | `imu` | 100 Hz | — | gyro σ 0.002 rad/s, bias σ 0.0005; accel σ 0.02 m/s², bias σ 0.005 |
| Wheel encoders | ros2_control state interfaces | 50 Hz | position and velocity of each drive wheel | physics |
| Fork height / load | ros2_control state interfaces | 50 Hz | lift position, velocity, **effort** | physics |

The IMU covariances in the ROS message come from the noise model (σ²: gyro 4e‑6, accel 4e‑4).
The orientation covariance is 0. This is satisfactory, because the EKF does not use the
orientation (refer to [system_design.md](system_design.md) D7).

## Topic table

| Topic | Type | Publisher | Main subscribers | QoS / rate (verified) |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `gz_bridge` | all nodes (`use_sim_time`) | ~1 kHz |
| `/scan` | `sensor_msgs/LaserScan` | `gz_bridge` | SLAM/AMCL, costmaps | best effort is satisfactory; 10 Hz, `lidar_link` |
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8) | `gz_bridge` | perception | **reliable, depth 1–2**; 10 Hz, `camera_optical_frame` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `gz_bridge` | perception | 10 Hz; fx = fy = 914.0, cx = 640, cy = 360 |
| `/imu/data` | `sensor_msgs/Imu` | `gz_bridge` | `ekf_filter_node` | best effort is satisfactory; 100 Hz, `imu_link` |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `robot_state_publisher`, `lift_controller` | 50 Hz |
| `/forklift_base_controller/odom` | `nav_msgs/Odometry` | `diff_drive_controller` | `ekf_filter_node` | 50 Hz, `odom` → `base_footprint` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` | Nav2, tests | 30 Hz |
| `/cmd_vel/teleop`, `/cmd_vel/approach`, `/cmd_vel/nav` | `geometry_msgs/TwistStamped` | operator / approach controller / Nav2 | `twist_mux` | priority 100 / 50 / 10, timeouts 0.5 / 0.3 / 0.5 s |
| `/estop` | `std_msgs/Bool` | operator / safety | `twist_mux` (lock, priority 255, no timeout) | `true` = stopped |
| `/forklift_base_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `twist_mux` | `diff_drive_controller` (0.5 s dead-man) | |
| `/fork/set_height` | action `forklift_interfaces/SetForkHeight` | `lift_controller` | task manager, tests | |
| `/fork/state` | `forklift_interfaces/ForkState` | `lift_controller` | `load_handler`, task manager | 20 Hz; `estimated_load_kg` from the lift effort |
| `/handling/pickup`, `/handling/place` | actions `PickupLoad`, `PlaceLoad` | `load_handler` | task manager, tests | |
| `/ground_truth/loads/<load>` | `geometry_msgs/PoseStamped` | `load_truth_bridge` | **only tests** | load poses at 30 Hz (teleport and support checks) |
| `/fork_lift_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | only `lift_controller` | `joint_trajectory_controller` | |
| `/scan_filtered` | `sensor_msgs/LaserScan` | `scan_footprint_filter` | AMCL, slam_toolbox, costmaps, collision monitor | 10 Hz; returns in the footprint are set to NaN |
| `/perception/qr_detections` | `forklift_interfaces/QrDetectionArray` | `qr_detector` | task manager, tests | one for each processed frame (≤ 10 Hz), an empty array if no code is visible |
| `/perception/qr_debug` | `sensor_msgs/Image` | `qr_detector` | RViz / persons | rendered only when there is a subscriber |
| `/ground_truth/odom` | `nav_msgs/Odometry` | `gz_bridge` | **only tests** (a test makes sure of this) | 50 Hz, frames `ground_truth_world`/`ground_truth_base` |

### Why camera images use a RELIABLE subscription
A 1280×720 RGB frame is 2.7 MB. Thus DDS divides it into many fragments. With best-effort
delivery, the loss of one fragment causes the loss of the full frame. Measured on the test
computer, with Gazebo at a constant 10.00 Hz: a reliable subscriber received **10.00 Hz**, and
a best-effort subscriber received only **1.58 Hz**. Thus the image subscribers use RELIABLE
with a queue depth of 1–2. The subscriber always gets the latest frame, and no backlog occurs.
Small messages (`/scan`, `/imu/data`) do not have this problem and stay best effort.

## Wheel odometry and sensor fusion
* `diff_drive_controller` calculates the odometry from the encoder feedback
  (`open_loop: false`). It does not publish TF (`enable_odom_tf: false`). A test checks the
  parameter and makes sure that the controller sends no data on `/tf`.
* `ekf_filter_node` (robot_localization, `two_d_mode`) uses the odometry vx, vy (= 0,
  non-holonomic) and vyaw, and the IMU vyaw. It is the only publisher of
  `odom → base_footprint`.
* The acceleration states have no measurement, so their process noise is ≈ 0. With the default
  values, the EKF position sometimes moved 0.3–0.5 m to the rear at the first motion (refer to
  [navigation.md](navigation.md)).
* `smooth_lagged_data` is on, and the subscription queues hold 1 s or more of data. Late
  wheel-odometry bursts (approximately 350 ms) go into the filter by a rewind. The filter or a
  full queue does not drop them. A regression test in `forklift_localization` checks this.

### Measured accuracy (Gazebo, compared with ground truth for evaluation only)

| Motion | Ground truth | Wheel odometry | EKF |
|---|---|---|---|
| 4 s at 0.5 m/s, then stop | 1.931 m | 1.930 m | 1.932 m |
| Turn in position, 3 s at 0.5 rad/s | 98.5° | 98.5° | 98.6° |

The automated test (`test_sim_motion.py`) requires agreement in 5 cm and 1–2° for straight
motion and turns.

**Contact-model finding.** With cylinder wheel collisions, DART put the contact at the outer
edge of the rim. Thus the effective track was 0.92 m, not 0.80 m. The wheel odometry showed
turns 13% too large (98.5° against 87.0° true). The EKF stayed at 87.2°, because the gyro
controls the yaw rate. This shows why IMU fusion is important. A test with narrow wheels
(0.04 m width gave an effective track of 0.84 m) confirmed the cause. Now the drive wheels use
sphere collisions (the visual is still a cylinder), and the error is gone.

## Self-occlusion and blind areas (by design)
* When the forks are below approximately 0.11 m, the LiDAR returns at ±61° are 0.17–0.19 m.
  These returns are the fork shanks. The scan filter removes them for the costmaps.
* The front LiDAR does not see the area behind the robot (±95° FOV). The **rear LiDAR**
  (`/scan_rear`, frame `rear_lidar_link`, on each forklift) monitors that sector and guards all
  reverse motion: `forklift_control/rear_safety.py`. Refer to
  [multi_robot_coordination.md](multi_robot_coordination.md#rear-lidar). A unit test makes
  sure that no ray of the rear LiDAR goes through the robot body.
* The camera sees the fork tips at the bottom of the image. This is intentional, for the
  alignment.

## Do the checks yourself
```bash
ros2 launch forklift_bringup sim.launch.py            # headless
ros2 topic hz /scan                                    # 10 Hz
ros2 topic echo --once /camera/camera_info
ros2 run tf2_tools view_frames                         # TF tree PDF
colcon test --packages-select forklift_test && colcon test-result --verbose
```
