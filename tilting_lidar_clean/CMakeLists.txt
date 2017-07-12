cmake_minimum_required(VERSION 2.8.3)
project(spin_hokuyo)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros laser_geometry geometry_msgs std_msgs tf sensor_msgs)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tilter
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS
     PCL
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tilt_motor tilt_motor_single_sweep_continuous.cpp)
target_link_libraries(tilt_motor ${catkin_LIBRARIES})

add_executable(tilt_transform tilt_transform.cpp)
target_link_libraries(tilt_transform ${catkin_LIBRARIES})

add_executable(scan_to_pcl scan_to_pcl.cpp)
target_link_libraries(scan_to_pcl ${catkin_LIBRARIES})

add_executable(pcl_assembler_client pcl_assembler_client.cpp)
target_link_libraries(pcl_assembler_client ${catkin_LIBRARIES})

add_executable(tilt_motor_subscribe tilt_motor_single_sweep_subscribe.cpp)
target_link_libraries(tilt_motor_subscribe ${catkin_LIBRARIES})

add_executable(hokuyo_robot_filter hokuyo_ground_filter.cpp)
target_link_libraries(hokuyo_robot_filter ${catkin_LIBRARIES})
target_link_libraries(hokuyo_robot_filter ${PCL_LIBRARIES})

add_executable(floor_max_z floor_max_z.cpp)
target_link_libraries(floor_max_z ${catkin_LIBRARIES})
target_link_libraries(floor_max_z ${PCL_LIBRARIES})

add_executable(hokuyo_ground_filter hokuyo_ground_filter.cpp)
target_link_libraries(hokuyo_ground_filter ${catkin_LIBRARIES})
target_link_libraries(hokuyo_ground_filter ${PCL_LIBRARIES})

add_executable(commander tutorials/node/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
