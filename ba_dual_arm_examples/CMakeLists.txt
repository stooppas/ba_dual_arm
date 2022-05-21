cmake_minimum_required(VERSION 3.5)
project(ba_dual_arm_examples)



## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

ament_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

add_executable(eef_align src/eef_align.cpp)

## Specify libraries to link a library or executable target against
ament_target_dependencies(eef_align 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
  Eigen3
)

add_executable(a_bot_spin src/a_bot_spin.cpp)

ament_target_dependencies(a_bot_spin 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
  Eigen3
)

add_executable(collision_test src/collision_test.cpp)

ament_target_dependencies(collision_test 
  moveit_core
  moveit_ros_planning_interface
  controller_manager
  rclcpp
  Eigen3
)


#############
## Install ##
#############
install(TARGETS eef_align a_bot_spin collision_test
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

