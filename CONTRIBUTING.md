# Contributing

## Procedure
1. Run `source scripts/env.sh`.
2. Keep each change small. If possible, change only one package.
3. Run `colcon build --symlink-install --packages-up-to <pkg>`.
4. Run `colcon test --packages-select <pkg> && colcon test-result --verbose`.
5. If the behavior changes, update the related `docs/` page and the README.

## Rules
* The autonomy packages must not use simulator internals (entity names, ground-truth topics,
  gz plugins). Only `forklift_simulation` (and the `sim_backend` xacro include) can contain
  simulator-specific items.
* Only tests can use ground truth (`/ground_truth/*`).
* Do not write fixed numbers in the nodes. Use ROS parameters from the `config/` directory of
  the package.
* Small sensor streams at a high rate (`/scan`, `/imu/data`) use `qos_profile_sensor_data`
  (best effort). **Camera images use RELIABLE with depth 1–2.** DDS divides a 2.7 MB image
  into fragments. With best-effort delivery, the loss of one fragment causes the loss of the
  full frame (measured: 1.6 Hz received of 10 Hz published).
* Before you start a simulation, stop the active simulation with `scripts/stop_sim.sh`. Two
  simulations on the same ROS domain or Gazebo partition corrupt the data of each other.
* Do not report a test as passed if its simulator did not operate. Use `pytest.skip` and give
  a reason.

## Style
* Python: PEP 8. The `ament_flake8` and `ament_pep257` tests of each package check it.
* Text: the README files, the docs, the code comments and the docstrings use ASD-STE100
  Simplified Technical English, Issue 9.
* Commits: subject in the imperative, `<package>: <change>` (for example
  `forklift_control: add lift timeout`).
