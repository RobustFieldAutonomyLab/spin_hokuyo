# spin_hokuyo
This repository contains code to control a Dynamixel motor and a 2D Hokuyo LiDAR to create a 3D point cloud and occupancy map.<br />

# Setup Directions
1. Install Dynamixel Drivers from ROS:<br/>
```sudo apt-get install ros-indigo-dynamixel-motor```<br/>
Install Hokuyo Node:
```sudo apt-get install ros-indigo-hokuyo-node```
Install Laser Assembler:
```sudo apt-get install ros-indigo-laser-assembler```
2. Download this repocitory into your src file.
3. In the motors_base/launch folder, update the dynamixel_servos.yaml to match your servo.
For information on how to fill out the .yaml file, please see dynamixel_servos_basic.yaml under motors_base/launch.
4. catkin_make and source the files, then:
```roslaunch motors_base basic_motors.launch```
The motor controller and spawner should properly generate topics that correspond to the motor. (i.e. no red text)
For errors, please see below.
5. In a new terminal, open the rostopics list.  There should be two new commands: /tilt_controller/command and /tilt_controller/state.  /command will issue new positions to the motor via Float64 (try this using rostopic pub).  /state will list various pieces of information about the motor (rostopic echo).
6. If this is fully functional, kill the node and do:
```roslaunch tilting_lidar_clean tilting_lidar_continuous.launch```
This should cause the motor to regularly sweep from 90 to -90 and generate a point cloud and occupancy map.
Be sure to update the dynamixel_servos_tilting.yaml for position, speed, etc.

# Possible Dynamixel Errors
Is the USB properly inserted and setup for your computer?  
Is the Dynamixel connected to a 12V power source via the adapter?  
Is the switch on the USB adapter at the proper loaction?  

# Source Repos
## Dynamixel Controllers
dynamixel_controllers is a package contained within the dynamixel_motor package, which is hosted at:  
https://github.com/arebgun/dynamixel_motor  
##
