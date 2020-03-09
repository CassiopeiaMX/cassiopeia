cmake_minimum_required(VERSION 2.8.3)
project(cassiopeia)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        geometry_msgs
        std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)