cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_controller)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  turtlesim
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_listener src/subscriber.cpp)
target_link_libraries(turtle_listener ${catkin_LIBRARIES})

add_executable(task5 src/exercise5.cpp)
target_link_libraries(task5 ${catkin_LIBRARIES})