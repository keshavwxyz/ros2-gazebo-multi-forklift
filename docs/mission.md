# Mission Manager

`forklift_task_manager` does the full autonomous mission. It identifies each box from its QR
code, moves the box to the destination that the code gives, and then moves the robot back to
home.

```bash
source scripts/env.sh && source install/setup.bash
ros2 launch forklift_bringup sim.launch.py start_mission:=true          # the mission starts immediately
# or start and control the mission manually:
ros2 launch forklift_bringup sim.launch.py rviz:=true
ros2 action send_goal --feedback /mission/run forklift_interfaces/action/RunMission "{}"
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission "{box_ids: [BOX_C]}"
ros2 service call /mission/pause  std_srvs/srv/Trigger
ros2 service call /mission/resume std_srvs/srv/Trigger
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"     # emergency stop
ros2 topic echo /mission/status
```

Each mission writes a JSON record (states, results, QR tasks, deliveries, failure reason) to
`~/.ros/forklift_missions/`.

## Architecture

| Layer | Owner | Function |
|---|---|---|
| State machine | `mission_fsm.py` (pure Python) | transitions, retries, failure policy, pause / e-stop, mission record |
| Executor | `mission_node.py` (`mission_manager`) | does each state with ROS: Nav2, perception, skills; timeouts |
| Skills | `forklift_control/load_handler.py` | pickup and placement |
| Navigation | Nav2 | travel between staging poses; no task logic |

The state machine does no I/O. Thus unit tests can simulate the results and check each
transition and each recovery path (`test/test_mission_fsm.py`, 24 tests). This includes paths
that are difficult to cause in Gazebo.

## States

```
IDLE → INITIALIZE → CHECK_SYSTEMS → LOCALIZE → SELECT_NEXT_BOX
  ┌─→ NAVIGATE_TO_PICKUP → DETECT_AND_IDENTIFY_BOX → PICKUP → NAVIGATE_TO_DESTINATION
  │     → PLACE_BOX → UPDATE_MISSION_STATE → SELECT_NEXT_BOX ─┐
  └──────────────────────── next box ─────────────────────────┘
SELECT_NEXT_BOX (no box) → RETURN_HOME → MISSION_COMPLETE | MISSION_FAILED
any → PAUSED / EMERGENCY_STOP → (resume) the same state again;  any → CANCELED
```

During the skills, `/mission/status` shows smaller phases from the skill feedback:
`APPROACH_BOX`, `ALIGN_FORKS`, `PICKUP`, `VERIFY_PICKUP`, `ALIGN_FOR_PLACEMENT`, `PLACE_BOX`,
`VERIFY_PLACEMENT`.

| State | Success condition | Failure codes |
|---|---|---|
| INITIALIZE | The Nav2, pickup, place and lift action servers are available | SYSTEMS_UNAVAILABLE |
| CHECK_SYSTEMS | New data on `/scan`, `/imu/data`, `/odometry/filtered`, `/fork/state` and QR detector output | SENSOR_UNAVAILABLE |
| LOCALIZE | `/amcl_pose` variance < 0.05 m² / 0.05 rad², and TF map→base_footprint is available | LOCALIZATION_FAILED |
| NAVIGATE_TO_PICKUP | Nav2 gets to the pickup staging pose of the box | NAVIGATION_FAILED, TIMEOUT |
| DETECT_AND_IDENTIFY_BOX | A **valid** QR detection of the selected box. Its decoded destination becomes the task. | BOX_NOT_VISIBLE, QR_UNREADABLE, IDENTITY_MISMATCH |
| PICKUP | PickupLoad is successful (identity, alignment, lift and weight are verified) | the error codes of the skill |
| NAVIGATE_TO_DESTINATION | Nav2 (loaded footprint) gets to the staging pose of the decoded destination | NAVIGATION_FAILED, DESTINATION_INVALID |
| PLACE_BOX | PlaceLoad is successful (released, moved back empty, the camera sees the box at the destination) | the error codes of the skill |
| RETURN_HOME | Nav2 gets to home | NAVIGATION_FAILED |

**Perception gives the task.** The mission order (BOX_A, BOX_B, BOX_C) and the pickup
locations come from the mission configuration. The destination comes from the decoded QR code
of the box. The perception node compares the payload with the configuration and refuses
unknown boxes or destinations. The autonomy code does not use simulator names or poses. The
test `test_no_simulator_shortcuts` makes sure of this.

## Recovery policy

* **Retries** (`config/mission_manager.yaml`): for temporary failure codes (navigation failure,
  box not visible, alignment, pickup not verified, …), the mission does the same state again.
  The limit is 2 retries for navigation and detection, and 1 retry for the skills. The counter
  goes back to zero when the state is successful.
* **Skip the box:** if the robot cannot pick up a box (box not found, incorrect identity, no
  more retries) and the forks are empty, the mission marks the box as failed and goes to the
  next box. The robot still goes back to home, but the result is MISSION_FAILED, with a list of
  the boxes that are not delivered.
* **Stop:** the mission stops immediately with MISSION_FAILED in these conditions: any failure
  while the forks hold a load, and any failure that gives an unknown load state (LIFT_FAILED,
  INSERTION_BLOCKED, PLACEMENT_NOT_VERIFIED, NOT_CARRYING, a skill timeout). The robot stays in
  its position for an operator. It is not safe to continue with an unknown load state.
* **Pause:** during navigation and detection, the pause is immediate. The mission cancels the
  Nav2 goal, the robot stops, and the state starts again when you resume. During pickup and
  placement, the pause waits until the skill is complete. A stop with the forks half in a
  pallet is more dangerous than the completion of a short, supervised manipulation.
* **Emergency stop** (`/estop` true): twist_mux immediately stops all motion, and the mission
  cancels the active action. To resume, send `/estop` false and then `/mission/resume`. After
  an interrupted pickup or placement, the mission refuses to resume (MISSION_FAILED, operator
  check), because the load state is not known.
* **Cancel** (action cancel): the mission cancels the active action, and the result is
  CANCELED.

## Results (Gazebo, `test_sim_mission.py` and manual runs)

| Run | Result | Placement error to the destination (ground truth) | Home | Time |
|---|---|---|---|---|
| manual 1 (before the lift solution) | MISSION_FAILED at BOX_B pickup: `LIFT_FAILED` (correct safe stop) | BOX_A 1.5 cm | n/a | 199 s sim |
| manual 2 | MISSION_COMPLETE 3/3 | 4.7 / 3.1 / 0.5 cm | 3.6 cm | 591 s sim, 614 s wall |
| suite 1 (with pause/resume) | MISSION_COMPLETE 3/3; **test failed**: BOX_B 10.7 cm > 10 cm limit | 3.1 / 10.7 / not shown | 2.3 cm | 620 s wall |
| suite 2 (with pause/resume) | MISSION_COMPLETE 3/3; test passed | 3.7 / 2.2 / 0.7 cm | 2.7 cm | 591 s wall |

In each completed run:
* The destinations came from the decoded QR payloads (BOX_A→SHELF_01, BOX_B→SHELF_03,
  BOX_C→SHELF_02).
* Each box was level (tilt 0.0°) at z = 0.250 m on its platform.
* The maximum box speed was 0.46 m/s (robot limit 0.4 m/s; no teleport).
* A pause during travel stopped the robot (0.000 m/s) in 4 s, and the resume continued the
  same leg.

The JSON mission records are in `~/.ros/forklift_missions/`.
