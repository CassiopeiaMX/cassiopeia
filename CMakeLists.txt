cmake_minimum_required(VERSION 2.8.3)
project(cassiopeia)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        image_transport
        cv_bridge
)
find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_driver src/camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES})
target_link_libraries(camera_driver ${OpenCV_LIBRARIES})