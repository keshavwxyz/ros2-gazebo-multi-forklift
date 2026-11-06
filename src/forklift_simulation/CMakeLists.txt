cmake_minimum_required(VERSION 3.8)
project(forklift_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models scripts urdf worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_simulation_assets test/test_simulation_assets.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
  ament_add_pytest_test(test_load_models test/test_load_models.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_package()
