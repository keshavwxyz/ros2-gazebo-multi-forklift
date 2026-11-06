cmake_minimum_required(VERSION 3.8)
project(forklift_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ekf_lagged_odometry.py TIMEOUT 60 ENV ROS_DOMAIN_ID=115)
endif()

ament_package()
