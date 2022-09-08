cmake_minimum_required(VERSION 3.8)
project(cliser)

# we don't have
set(BUILD_TESTING OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)

# install the python modules:
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  cliser/nodes/minimal_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# create the target:
add_executable(minimal_service
  src/minimal_service.cc
)

# add the dependencies (MUST INCLUDE RCLCPP):
ament_target_dependencies(minimal_service

  # what do we depend on?
  rclcpp
  example_interfaces
)

# install the nodes:
install(TARGETS
  minimal_service
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()