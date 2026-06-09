# CMake generated Testfile for 
# Source directory: /home/chester_lnx/ws_6dof_robot_arm/ws_ros2/src/arm_pkg
# Build directory: /home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_publisher "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg/test_results/arm_pkg/test_publisher.gtest.xml" "--package-name" "arm_pkg" "--output-file" "/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg/ament_cmake_gtest/test_publisher.txt" "--command" "/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg/test_publisher" "--gtest_output=xml:/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg/test_results/arm_pkg/test_publisher.gtest.xml")
set_tests_properties(test_publisher PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg/test_publisher" TIMEOUT "60" WORKING_DIRECTORY "/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/build/arm_pkg" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/src/arm_pkg/CMakeLists.txt;63;ament_add_gtest;/home/chester_lnx/ws_6dof_robot_arm/ws_ros2/src/arm_pkg/CMakeLists.txt;0;")
subdirs("gtest")
