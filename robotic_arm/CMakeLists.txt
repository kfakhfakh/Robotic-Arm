cmake_minimum_required(VERSION 3.8)
project(robotic_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_control_interface REQUIRED)
add_executable(position src/test.cpp)
ament_target_dependencies(position rclcpp moveit_ros_planning_interface geometry_msgs )


install( 
  PROGRAMS
  src/camera_sim.py
  DESTINATION lib/${PROJECT_NAME}/
)

install( 
  TARGETS
  position
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY 
  urdf 
  rviz 
  launch 
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
