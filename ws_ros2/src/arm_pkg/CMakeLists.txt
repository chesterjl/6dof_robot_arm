cmake_minimum_required(VERSION 3.8)
project(arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)


# Note: Removed std_msgs since it isn't linked to any targets below

# Define executables
add_executable(plan_around_objects src/plan_around_objects.cpp)
add_executable(joint_state_publisher src/joint_state_publisher_node.cpp)
add_executable(arm_publisher_gazebo src/arm_publisher_gazebo.cpp)

# Target dependencies for main nodes
ament_target_dependencies(
  joint_state_publisher
  rclcpp
  sensor_msgs
)

ament_target_dependencies(
  arm_publisher_gazebo
  rclcpp
  trajectory_msgs
  sensor_msgs
  std_msgs
)

ament_target_dependencies(plan_around_objects
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  shape_msgs
)

# Install rules
install(TARGETS
  joint_state_publisher
  arm_publisher_gazebo
  plan_around_objects
  DESTINATION lib/${PROJECT_NAME}
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Register the GTest executable
  ament_add_gtest(test_publisher test/gtest/test_publisher.cpp)

  # Link target dependencies for the test
  ament_target_dependencies(
    test_publisher
    rclcpp
    trajectory_msgs
    sensor_msgs
  )

  # FIXED: Allow GTest to find "../../src/arm_publisher_gazebo.cpp" seamlessly
  target_include_directories(test_publisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

endif()

ament_package()