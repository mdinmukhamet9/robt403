cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()
include_directories(  ${catkin_INCLUDE_DIRS})

add_executable(talker src/publisher.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_executable(listener src/subscriber.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})


