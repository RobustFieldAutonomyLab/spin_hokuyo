cmake_minimum_required(VERSION 2.8.3)
project(spin_hokuyo)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS roscpp message_generation pcl_ros laser_geometry geometry_msgs std_msgs tf sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  dynamixel_driver
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  dynamixel_msgs
  control_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tilt_motor src/tilt_motor_single_sweep_continuous.cpp)
target_link_libraries(tilt_motor ${catkin_LIBRARIES})

add_executable(tilt_transform src/tilt_transform.cpp)
target_link_libraries(tilt_transform ${catkin_LIBRARIES})

add_executable(scan_to_pcl src/scan_to_pcl.cpp)
target_link_libraries(scan_to_pcl ${catkin_LIBRARIES})

add_executable(pcl_assembler_client src/pcl_assembler_client.cpp)
target_link_libraries(pcl_assembler_client ${catkin_LIBRARIES})

add_executable(tilt_motor_subscribe src/tilt_motor_single_sweep_subscribe.cpp)
target_link_libraries(tilt_motor_subscribe ${catkin_LIBRARIES})

add_executable(hokuyo_robot_filter src/hokuyo_robot_filter.cpp)
target_link_libraries(hokuyo_robot_filter ${catkin_LIBRARIES})

add_executable(commander tutorials/node/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
