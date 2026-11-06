# Load Handling: Pickup and Placement

Status: **complete and verified in Gazebo**. **Only the forks move the load**, through physical
contact and friction in DART. The simulator does not teleport the load and does not attach it
with a joint. A static test prevents both in the autonomy code. A sent command does not show
success. A sensor verifies each step.

## Interfaces
| Action | Type | Start position | End position |
|---|---|---|---|
| `/handling/pickup` | `forklift_interfaces/action/PickupLoad` (`box_id`) | the pickup staging pose of the box* | the same staging pose, with the load at transport height |
| `/handling/place` | `forklift_interfaces/action/PlaceLoad` (`box_id`, `destination_id`) | the staging pose of the destination*, with the load | the same staging pose, forks empty, at insertion height |

\* If the robot is more than 0.25 m or 10° from that staging pose, the skill first moves the
robot there with Nav2. If Nav2 fails, the result is `ERROR_NAVIGATION_FAILED`. The docking
controller follows a line and does not plan or avoid obstacles. Thus it refuses a segment that
is longer than 3 m. Before this limit, a `place` immediately after a `pickup` used the docking
controller for a 4.7 m travel and put the box 1.26 m from the target. With the limit, the same
request puts the box 3.8 cm from the target.

Each result has `success`, an `error_code` (constants in the Result section) and a message. The
feedback gives the phase and the alignment errors. The node is `forklift_control/load_handler`.
The parameters are in `config/load_handler.yaml`. The pure logic is in
`load_handling_logic.py`. Nav2 controls the travel between staging poses. The task manager
controls the sequence.

## Mechanism and heights
The load is one rigid body: a pallet (0.8 × 0.8 m, 0.15 m high, 0.12 m fork pockets) with a box.
The function `fork_heights` calculates all the lift set-points from the load dimensions and the
robot parameters:

| Phase | Lift q | Fork top | Reason |
|---|---|---|---|
| insert | 0.02 m | 0.07 m | The pocket is 0–0.12 m: 3 cm above the floor, 5 cm below the deck |
| transport | 0.25 m | 0.30 m | The pallet bottom is at 0.18 m, above the 0.12 m LiDAR plane |
| place (move in) | 0.37 m | 0.42 m | The pallet bottom is 5 cm above the 0.25 m platform |
| release | 0.27 m | 0.32 m | The platform holds the pallet. The forks are 5 cm below the deck and 3 cm above the platform. |

## Pickup sequence
0. If the forks already hold a load, refuse the request (`INVALID_REQUEST`). If necessary, move
   to the staging pose (refer to the text above).
1. Make sure that the robot is stationary (EKF twist ≈ 0 for 0.5 s).
2. Move the forks to the insertion height (SetForkHeight action).
3. **Identify:** get 5 or more QR detections of the load in front of the robot. Change each
   detection into a load pose in `odom` (TF at the image time, QR offset from the mission).
   Incorrect box → `IDENTITY_MISMATCH`. Only codes that do not decode → `QR_UNREADABLE`. No
   detection → `BOX_NOT_VISIBLE`.
4. **Verify the location:** the box must be in 0.30 m and 15° of its configured pickup (map
   frame).
5. **Align:** follow the axis of the load to the pre-insertion pose (fork tips 0.10 m before
   the pallet). Each new detection makes the target more accurate. When the robot is
   stationary, make sure that the lateral error is 3 cm or less and the heading error is 2° or
   less. If an error is more than this, move back 0.4 m and try again (2 retries). If the
   retries fail, the result is `ALIGNMENT_FAILED`.
6. **Insert:** move the last 1.08 m straight along the docking line at 0.06 m/s. Near the
   pallet, the QR code goes out of the camera view. Thus this step uses only odometry. If the
   robot stalls for 2 s, the result is `INSERTION_BLOCKED`.
7. **Lift** to the transport height. After 1 s, the load estimate from the lift effort must be
   0.5–2.0× the configured mass. If not, lower the forks, move out, and send
   `PICKUP_NOT_VERIFIED`.
8. Record the position of the load on the forks. The robot uses the camera estimate from before
   the insertion, in the base frame at the end of the insertion (odometry). The placement uses
   this pose, not the nominal pose. Limit: when the insertion goes to its end, this value is
   nominal (0.798–0.800 m in all runs), but ground truth shows 0.80–0.86 m. The robot measures
   what the odometry shows. Thus the correction helps only when an insertion stops before its
   end. A full solution needs a sensor on the forks, for example a contact switch on the load
   backrest.
9. Change the navigation footprint to *loaded*, move back to the staging pose, and send success.

## Placement sequence
1. Validate: the destination is present, the destination from the QR code of the box is the
   same (`DESTINATION_MISMATCH`), and the forks hold a load (`NOT_CARRYING`).
2. Lift the forks to the placement height.
3. Align on the destination pose to the pre-insertion pose. The robot changes the AMCL pose to
   `odom` one time.
4. **Verify the platform with the LiDAR:** the range to the front face must be in 0.12 m of the
   value from the map (`DESTINATION_NOT_FOUND`). The scan goes below the lifted pallet.
5. Move in (with stall detection), then lower the forks to the release height.
6. **Verify the release:** the load estimate must be less than 5 kg (the platform holds the
   weight). If not, lift the forks again and send `PLACEMENT_NOT_VERIFIED`.
7. Change the footprint to *unloaded* and move back to the staging pose. After this, the forks
   must still show less than 5 kg (this shows a pallet that the forks pulled). Then lower the
   forks to the insertion height. **Do not lower the forks onto the lower hard stop of the
   lift.** In Gazebo, if the robot moves when the joint is on its limit, the joint stays stuck,
   and the next pickup fails with `LIFT_FAILED`. The first full mission run found this problem
   (refer to [gazebo_setup.md](gazebo_setup.md)).
8. **Verify with the camera:** the QR code of the box must be in 0.15 m and 8° of the
   destination.

## Tolerances, limits, timeouts
| Item | Value | Basis |
|---|---|---|
| Lateral / heading error before insertion | 3 cm / 2° | 10 cm pocket clearance on each side, less 1.1 m of blind travel at 2° (≈ 4 cm) |
| Speeds | approach 0.12, insert 0.06, reverse 0.06 m/s | slow docking; the rear is blind |
| Approach steering gains | heading 2.0, lateral 5.0, ≤ 0.4 rad/s | removes 15 cm / 10° in 0.7 m (unit test) |
| Lift | limits from the URDF (0–1 m, 0.2 m/s), action timeout 30 s | lift_controller refuses goals out of range |
| Load check | 0.5–2.0 × mass to accept; < 5 kg means empty | effort − (110 kg × g), mass from the URDF |
| Detection / stationary / stall | 8 s / 5 s / 2 s | |

## Verified results (Gazebo, `test_sim_pickup_place.py`, last run)
| Check | Result |
|---|---|
| Identify BOX_A at PICK_01 | 1 cm from the configured position |
| Alignment before insertion | −0.5 cm, 0.2° (3 runs: ≤ 0.6 cm, ≤ 0.8°) |
| Load on the forks after the lift | 30.0 kg measured (true value 30 kg). The load is 0.82 m in front of the base (nominal 0.80), lateral ≤ 0.8 cm, pallet bottom at 0.180 m. |
| Transport (Nav2, 634 samples at 30 Hz) | load-on-forks drift 0.9 cm along / 1.5 cm across, height constant at 0.180 m, tilt 0.00° |
| Placement at SHELF_01 (ground truth) | 1.4 cm / 1.0° from the target, on the platform deck at z = 0.250 m, no movement after the release (5 runs: 1.4–5.9 cm, ≤ 1.2°) |
| Placement (camera check by the robot) | 2.5 cm / 1.1° |
| No teleport | 5,861 load poses; maximum load speed 0.45 m/s (robot limit 0.4) |
| Failure cases | `INVALID_REQUEST` (unknown box), `NOT_CARRYING`, `DESTINATION_MISMATCH` (no motion), `IDENTITY_MISMATCH` (BOX_B requested, BOX_A in front), `BOX_NOT_VISIBLE` (empty pickup) |

Unit tests: `test_load_handling_logic.py` (15 tests: geometry, controller convergence forward
and in reverse, heights, robust target, load window) and `test_config_consistency.py` (handler
geometry against the URDF). Static test: `test_no_simulator_shortcuts.py` (no teleport, no
attachment joints, no ground truth and no Gazebo load names in the autonomy code).

### Differences between runs
In seven pickups, the load center was 0.800–0.821 m in front of the base five times (nominal
0.80), and 0.855–0.864 m two times. In all cases, the load was fully on the forks (the
physical range is 0.78–0.98 m). One run recorded the load from the time of the pickup. It showed
a transport drift of only 1.0 / 1.5 cm. Thus **the pallet does not slide during transport**. The
difference is in the insertion depth: sometimes the blind last 1.08 m stops approximately 5 cm
before its end.

The placement error in all runs is 1.4–5.9 cm (tolerance 15 cm). It comes from the insertion
offset and from the AMCL-based placement alignment (localization error 3–8 cm). One run had a
nominal 0.804 m insertion, and the error was still 5.9 cm. Possible causes are the scatter of
the QR depth estimate at the pre-insertion pose and the odometry during the blind segment.
These causes are not isolated yet. Possible solutions are LiDAR measurement of the pallet front
or a last QR measurement nearer to the pallet.

## Conditions that the simulation tested only by chance, or did not test
* Gazebo showed these conditions, but no dedicated test does them: the **alignment retry**
  (BOX_C: heading 2.3° > 2°, the robot moved back, the second attempt was 0.9°, then the box was
  delivered; full-suite run 2026-10-29), **`DESTINATION_NOT_FOUND`** (LiDAR face check, the
  4.7 m direct-place case before the navigate-first change) and **`LIFT_FAILED`** (the lift
  stuck on its hard stop, before the solution). In each case the skill operated as designed: it
  tried again or stopped safely.
* These conditions did not occur in the simulation: `ALIGNMENT_FAILED` after all the retries,
  `INSERTION_BLOCKED`, `PICKUP_NOT_VERIFIED`, `WRONG_LOCATION`, `IDENTITY_MISMATCH` at a
  correct pickup (this world cannot put the incorrect box there), and a skill cancel during
  operation. Unit tests check the response of the *mission* to each of these codes
  (`forklift_task_manager/test/test_mission_fsm.py`).

## Limitations
* **Blind last insertion:** the last 1.1 m approximately uses only odometry. If the pallet moved
  after the last QR measurement, the forks meet it with an alignment error. Only the stall
  check finds a collision.
* **The placement alignment uses AMCL** (3–8 cm). Only the LiDAR face range checks it. The
  0.2 m lateral margin of the platform is sufficient for this error, but the platform has no
  visual fiducial.
* **One load type** (dimensions from the mission configuration). One pallet on each platform,
  one level.
* **Physics:** DART contact with μ 0.6–0.8 and low accelerations (≤ 0.4 m/s²). Harder braking
  or forks with no tilt on real hardware can cause different behavior. There is no mast tilt to
  hold the load.
