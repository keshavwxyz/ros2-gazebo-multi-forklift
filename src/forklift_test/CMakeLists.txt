cmake_minimum_required(VERSION 3.8)
project(forklift_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # Each test gets its own ROS domain and Gazebo partition. Thus, a test never receives data
  # from a different simulation on the same machine.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_no_simulator_shortcuts test/test_no_simulator_shortcuts.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 60)

  # Tier 2: no simulator (ros2_control mock hardware). These tests always run, also in CI.
  add_launch_test(test/test_mock_control.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=111)
  add_launch_test(test/test_mission_manager_node.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=118)
  add_launch_test(test/test_fleet_coordinator.py
    TIMEOUT 900
    ENV ROS_DOMAIN_ID=110)

  # Tier 3: Gazebo simulation. These tests need gz + GPU/EGL rendering. To disable them, use
  #   colcon test --cmake-args -DFORKLIFT_SIM_TESTS=OFF   (or build with that option)
  find_program(GZ_EXECUTABLE gz)
  option(FORKLIFT_SIM_TESTS "Run Gazebo simulation tests" ON)
  if(FORKLIFT_SIM_TESTS AND GZ_EXECUTABLE)
    add_launch_test(test/test_sim_sensors_tf.py
      TIMEOUT 240
      ENV ROS_DOMAIN_ID=112 GZ_PARTITION=forklift_test_sensors)
    add_launch_test(test/test_sim_motion.py
      TIMEOUT 300
      ENV ROS_DOMAIN_ID=113 GZ_PARTITION=forklift_test_motion)
    add_launch_test(test/test_sim_perception.py
      TIMEOUT 360
      ENV ROS_DOMAIN_ID=114 GZ_PARTITION=forklift_test_perception)
    add_launch_test(test/test_sim_navigation.py
      TIMEOUT 1200
      ENV ROS_DOMAIN_ID=116 GZ_PARTITION=forklift_test_navigation)
    add_launch_test(test/test_sim_pickup_place.py
      TIMEOUT 1200
      ENV ROS_DOMAIN_ID=117 GZ_PARTITION=forklift_test_pickup_place)
    add_launch_test(test/test_sim_rear_safety.py
      TIMEOUT 300
      ENV ROS_DOMAIN_ID=120 GZ_PARTITION=forklift_test_rear_safety)
    add_launch_test(test/test_sim_fleet.py
      TIMEOUT 3600
      ENV ROS_DOMAIN_ID=121 GZ_PARTITION=forklift_test_fleet)
    add_launch_test(test/test_sim_fleet_conflicts.py
      TIMEOUT 1800
      ENV ROS_DOMAIN_ID=122 GZ_PARTITION=forklift_test_fleet_conflicts)
    add_launch_test(test/test_sim_mission.py
      TIMEOUT 3000
      ENV ROS_DOMAIN_ID=119 GZ_PARTITION=forklift_test_mission)
  else()
    message(WARNING "Gazebo simulation tests NOT EXECUTED (gz found: '${GZ_EXECUTABLE}', "
                    "FORKLIFT_SIM_TESTS=${FORKLIFT_SIM_TESTS})")
  endif()
endif()

ament_package()
