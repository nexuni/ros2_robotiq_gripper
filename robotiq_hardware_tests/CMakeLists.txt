# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(robotiq_hardware_tests)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# Generate service and action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperControl.srv"
  "action/GripperControl.action"
  DEPENDENCIES action_msgs
)

add_executable(full_test
    src/gripper_interface_test.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(full_test robotiq_driver)

add_executable(hande_gripper_control_node
    src/hande_gripper_control_node.cpp
)
ament_target_dependencies(hande_gripper_control_node rclcpp rclcpp_action robotiq_driver)

add_executable(test_gripper_client
    src/test_gripper_client.cpp
)
ament_target_dependencies(test_gripper_client rclcpp)

add_executable(test_gripper_action_client
    src/test_gripper_action_client.cpp
)
ament_target_dependencies(test_gripper_action_client rclcpp rclcpp_action)

# Link the generated service and action interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hande_gripper_control_node "${cpp_typesupport_target}")
target_link_libraries(test_gripper_client "${cpp_typesupport_target}")
target_link_libraries(test_gripper_action_client "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  full_test
  hande_gripper_control_node
  test_gripper_client
  test_gripper_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
