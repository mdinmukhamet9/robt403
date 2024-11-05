cmake_minimum_required(VERSION 3.0.2)
project(lab4)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_core
  std_msgs
  message_generation
  moveit_ros_planning_interface
  tf2_ros
  rospy
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs moveit_core moveit_ros_planning_interface tf2_ros std_msgs
)

## Specify include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables and link them with necessary libraries
add_executable(test_moveit src/task2.cpp)
add_executable(test_rectangle src/task3.cpp)

## Link executables to catkin libraries
target_link_libraries(test_moveit ${catkin_LIBRARIES})
target_link_libraries(test_rectangle ${catkin_LIBRARIES})

## Add dependencies to ensure message headers are generated before compilation
add_dependencies(test_moveit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_rectangle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

