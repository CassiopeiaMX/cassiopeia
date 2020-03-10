cmake_minimum_required(VERSION 2.8.3)
project(cassiopeia)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        geometry_msgs
        std_msgs
        message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
    FILES
    Altitude.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

