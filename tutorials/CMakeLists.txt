cmake_minimum_required(VERSION 2.8.3)
project(tutorials)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dynamixel_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorials
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(commander commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})

