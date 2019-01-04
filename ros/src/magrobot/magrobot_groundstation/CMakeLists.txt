cmake_minimum_required(VERSION 2.8.3)
project(magrobot_groundstation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joy2twist_node src/joy2twist.cpp)
target_link_libraries(joy2twist_node ${catkin_LIBRARIES})
