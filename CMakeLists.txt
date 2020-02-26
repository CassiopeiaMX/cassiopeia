cmake_minimum_required(VERSION 2.8.3)
project(cassiopeia)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/camera_driver.cpp)
target_link_libraries(talker $(catkin_LIBRARIES))