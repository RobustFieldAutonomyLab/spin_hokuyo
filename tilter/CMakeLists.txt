cmake_minimum_required(VERSION 2.8.3)
project(tilter)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tilter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(tilter tilting.cpp)
target_link_libraries(tilter ${catkin_LIBRARIES})
