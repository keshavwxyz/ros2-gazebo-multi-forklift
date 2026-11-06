cmake_minimum_required(VERSION 3.8)
project(forklift_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python helpers (URDF kinematics for the tests and the tools)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # There are no C/C++ source files. The repository LICENSE applies to the copyright headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_model test/test_urdf_model.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock_control_interfaces.py TIMEOUT 90)
endif()

ament_package()
