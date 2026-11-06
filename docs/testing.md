# Testing

```bash
source scripts/env.sh
colcon build --symlink-install
colcon test && colcon test-result --verbose          # all tiers (~100 min with Gazebo)
colcon test --cmake-args -DFORKLIFT_SIM_TESTS=OFF   # skip Gazebo (CI)
# one Gazebo test, with its own ROS domain and Gazebo partition:
cd build/forklift_test && ctest -R test_sim_mission --output-on-failure
```

Each Gazebo test operates in its own ROS domain (111–122) and Gazebo partition. Thus a test
does not see the data of a different simulation. launch_testing writes subtest failures only
to the raw log (`build/<pkg>/Testing/Temporary/LastTest_*.log`; search for `^FAIL:`). It does
not always write them to the XML.

## Tiers

| Tier | Content | Location | Necessary items |
|---|---|---|---|
| 1 Unit | pure logic: kinematics, lift profiles, load-handling geometry, QR codec / detection / pose, mission configuration, **mission state machine** (24), navigation configuration, simulator assets | `test/` of each package | none |
| 1 Lint | flake8, pep257, xmllint, lint_cmake | all packages | none |
| 1 Guard | `test_no_simulator_shortcuts`: the autonomy code must not use simulator ground truth, entity names, teleport or attach | `forklift_test` | none |
| 2 Integration | mock ros2_control (drive, lift, e-stop), EKF with lagged odometry, QR node on synthetic images, **mission manager node** without servers | `forklift_test`, packages | only ROS |
| 3 Simulation | sensors and TF, motion, perception, navigation, pickup/place, **full mission**, rear safety, **two-forklift fleet and conflicts** | `forklift_test/test/test_sim_*.py` | Gazebo + GPU |

## Latest results (2026-10-29, multi-forklift version, last code)

One full run, `colcon test` on a clean build: **489 tests, 0 failures, 0 errors.**

| Test | Result | Main values |
|---|---|---|
| Unit + lint + guard (all packages) | PASS | includes rear-safety logic (11), fleet registry / allocator / conflicts / peer outline (33) |
| `test_mock_control`, `test_mission_manager_node`, EKF, QR node | PASS | |
| `test_fleet_coordinator` (7 fake-robot scenarios) | PASS | claims, robot loss, ack timeout, zones, path yield, deadlock, rear-blocked |
| `test_sim_sensors_tf` (14) | PASS | lift held at 0.02 m |
| `test_sim_motion` (8) | PASS | |
| `test_sim_perception` (5) | PASS | |
| `test_sim_navigation` (8) | PASS | localization median 3.9 cm / maximum 8.2 cm, yaw maximum 0.90° (741 samples) |
| `test_sim_pickup_place` (12) | PASS | transport drift ≤ 1.4 cm; placed 3.7 cm from SHELF_01 (ground truth) |
| `test_sim_rear_safety` (6) | PASS | rear scan 10.0 Hz, 181 rays; reverse stopped with a 0.43 m gap (STOP 0.35 m) |
| `test_sim_mission` (5) | PASS | 3/3: 3.8 / 4.5 / 4.1 cm; home in 2.5 cm |
| `test_sim_fleet` (5) | PASS | 2 robots, 3/3 in 378 s sim; 4.7 / 5.3 / 5.2 cm; one owner for each box; 0 overlap; home 3.0 cm / 2.4° and 7.6 cm / 2.6° |
| `test_sim_fleet_conflicts` (3) | PASS | rear-to-rear gap 0.34 m, the two robots STOP; head-on: 3 HEAD_ON conflicts solved in ≤ 1 s, the two robots at home, 0 overlap |

The Gazebo tests use the ROS domains 111–122. The ground-truth overlap check of the fleet tests
includes the body, the rear LiDAR housing and the fork tines.

## Failures found and solved in the multi-forklift version

| Symptom | Cause | Solution |
|---|---|---|
| lift locked at 0.000 m (4/4) after the rear LiDAR link was added | unknown Gazebo interaction; bisection found the link | spawn at 0.02 m, lift_controller lifts the forks at startup |
| INSERTION_BLOCKED at placement | the mast stopped ~1 cm from the platform face | the insertion stops at the face that the LiDAR measures (4 cm margin) |
| the rear LiDAR housing was outside the Nav2 footprint | rear edge of the footprint −1.18 m, the housing goes to −1.23 m | footprint −1.25 m, rear safety measured from −1.23 m |
| robot never available / QR topic missed / AMCL never published map→odom | DDS discovery misses at intervals (~70 nodes) | watchdogs subscribe again, CHECK_SYSTEMS subscribes again, lifecycle_starter cycles AMCL |
| 3 navigation retries in 0.2 s against an old map→odom | no delay between the retries | 2 s retry_delay |
| the robot in the conflict test could not turn | test pose 1.3 m from a column (fork swing 1.44 m) | the test geometry is moved |
| head-on livelock (> 10 min) and possible fork-to-fork contact | the robot that gave way waited in the home zone of the other robot; the LiDAR cannot see the tines; "carrying" came from a sensor value that changed quickly | zone-clearing priority rule, peer outlines in the costmaps, "carrying" from the task state |
| a failed navigation to home was not tried again | the coordinator ignored the result, "home" = in 0.6 m | 0.25 m tolerance, 3 retries |

## Failures found and solved in the single-forklift mission

| Symptom | Cause | Solution | Evidence |
|---|---|---|---|
| `place` immediately after `pickup` put the box 1.26 m from the target | the docking controller moved 4.7 m (no planning) | the skills first move to the staging pose with Nav2; 3 m docking limit | same request: 3.8 cm |
| 2nd box: `LIFT_FAILED` | in Gazebo, a lift on its lower hard stop gets stuck when the robot moves | park at the insertion height | occurred 2/2; 0/2 when parked at 0.02 m |
| the bringup stopped, the test did not start | `planner_server` had a segfault in configure (in Nav2) | respawn the Nav2 servers, lifecycle_starter monitors them | SIGSEGV sent: recovered 3/3, then navigation was successful |
| BOX_B placed 10.7 cm from the target (limit 10) | variation: AMCL (≤ 8 cm) + insertion depth (≤ 6 cm) | **not fully solved**, refer to the open problems | next two runs: ≤ 3.7 cm |
| pickup accepted with a load on the forks | no guard | `INVALID_REQUEST` | test_8 |
| test_7 thought that the robot points to an incorrect box | not correct after the navigate-first change | new test: remote pickup | test_7 |

## Not done / not covered

* **Never occurred in the simulation.** The code paths have timeouts and safe stops, and unit
  tests check the response of the mission: `INSERTION_BLOCKED`, `PICKUP_NOT_VERIFIED`,
  `WRONG_LOCATION`, `IDENTITY_MISMATCH` at a correct pickup, `ALIGNMENT_FAILED` after all the
  retries, skill cancel during operation, emergency stop *during* a mission (mock and Gazebo
  tests check the twist_mux e-stop, but only unit tests check the e-stop response of the
  mission), mission cancel in Gazebo, AMCL crash.
* **Repeatability:** after the lift solution, the full mission delivered 3/3 boxes and went
  back to home in all 3 Gazebo runs (one manual run, two suite runs). The mission *test* passed
  in 2 of them. The third run failed its 10 cm placement limit (10.7 cm). The run before the
  lift solution failed (`LIFT_FAILED` on the second box). There are no long-run statistics and
  no random start poses.
* **Multi-forklift, not done in Gazebo:** three or more robots; a robot that reverses into a
  different robot during a mission (the REAR_BLOCKED coordination path; only fake robots and the
  rear-to-rear Gazebo test of the safety layer cover it); robot loss, ack timeouts and deadlock
  recovery (only fake robots); HELD task recovery.
* **Multi-forklift repeatability:** the two-robot mission completed 3/3 in the last 4 runs
  (three test runs, one manual run). Previous development runs failed for the causes in the
  table above.
* **Hardware:** no tests.

## Open problems

1. **Placement accuracy: 0.5–10.7 cm in 19 placements, measured against ground truth**
   (median ≈ 3 cm; one is more than the 10 cm limit of the mission test). The causes are AMCL
   (3–8 cm at the shelves) and the insertion depth (ground truth 0.80–0.86 m on the forks). The
   estimate of the robot for the load on the forks is nominal, because it comes from the
   odometry of the commanded motion. A fiducial on each platform, or a contact sensor on the
   fork backrest, can remove most of this error. The ±20 cm margin of the platform holds the
   load safely in all the observed cases.
2. **Nav2 `planner_server` segfault** (1 in ~40 launches, cause not known). Now the system
   recovers automatically, and the test output shows a WARNING.
3. **Gazebo shutdown stop** (SIGKILL necessary, at intervals); the test output shows a
   WARNING. One time, `parameter_bridge` had a segfault (−11) at shutdown after a mission test
   that passed.
4. **Two-robot graph:** DDS discovery misses and rclpy take errors at intervals; decreased, but
   the cause is not known (refer to [multi_robot_coordination.md](multi_robot_coordination.md)).
5. **Home parking:** 3–8 cm / ~2.5° from home (ground truth). AMCL in the home area, which has
   few features, and the 8 cm / 3.4° goal tolerance limit the accuracy.
