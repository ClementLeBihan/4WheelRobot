cmake_minimum_required(VERSION 2.8.3)
project(magrobot_platform)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(magrobot_node src/main.cpp)
target_link_libraries(magrobot_node ${catkin_LIBRARIES})
