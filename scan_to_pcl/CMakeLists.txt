cmake_minimum_required(VERSION 2.8.3)
project(scan_to_pcl)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros laser_geometry geometry_msgs)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(scan_to_pcl scan_to_pcl.cpp)
target_link_libraries(scan_to_pcl ${catkin_LIBRARIES})
