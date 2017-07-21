# spin_hokuyo
This repository contains code to control a Dynamixel motor and a 2D Hokuyo LiDAR to create a 3D point cloud that can be visualized in rviz.  This point cloud can then be used to create an octomap (code for this is not included in this repository).<br />
Created By: Sarah Bertussi and Paul Szenher</br>

# Setup Directions
1. Install Dynamixel Drivers from ROS:<br/>
```sudo apt-get install ros-indigo-dynamixel-motor```<br/>
Install Hokuyo Node: <br/>
```sudo apt-get install ros-indigo-hokuyo-node```<br/>
Install Laser Assembler:<br/>
```sudo apt-get install ros-indigo-laser-assembler```<br/>
2. Download this repocitory into your src file.
3. In the motors_base/launch folder, update the dynamixel_servos.yaml to match your servo.<br/>
For information on how to fill out the .yaml file, please see dynamixel_servos_basic.yaml under motors_base/launch.
4. catkin_make and source the files, then:<br/>
```roslaunch spin_hokuyo basic_motors.launch```<br/>
The motor controller and spawner should properly generate topics that correspond to the motor. (i.e. no red text)
5. In a new terminal, open the rostopics list.  There should be two new commands: /tilt_controller/command and /tilt_controller/state.  /command will issue new positions to the motor via Float64 (try this using rostopic pub).  /state will list various pieces of information about the motor (rostopic echo).
6. If this is fully functional, kill the node and do:<br/>
```roslaunch spin_hokuyo tilting_lidar_continuous.launch```<br/>
This should cause the motor to regularly sweep from 90 to -90 and generate a point cloud and occupancy map.
Be sure to update the dynamixel_servos_tilting.yaml for position, speed, etc.

## Possible Dynamixel Errors
Is the USB properly inserted and setup for your computer?  
Is the Dynamixel connected to a 12V power source via the adapter?  
Is the switch on the USB adapter at the proper loaction?  

# Acknowledgements

Inspired by the work seen here: </br>
https://github.com/gcc-robotics/3d_photobooth/blob/master/CapstoneFinalReport_VisionTeam.pdf </br>

## Dynamixel Interface
The code used to interact with the Dynamixel comes from bot the ROS Drivers Install and the dynamixel_motor package, which is hosted at:  
https://github.com/arebgun/dynamixel_motor  

These files are in the /dynamixel folder under /src, the /srv folder, and the setup.py file.  The format for the .yaml files also came from this package
##
## Dynamixel Control
The class system used to control the Dynamixel motors and the basis for the servo template were based on the examples provided in Chap. 8 of "Effective Robotics Programming with ROS - Third Edition" by Anil Mahtani, Luis Sanchez, Enrique Fernandea, and Aaron Martinez.
##
## pcl_assembler_client
The pcl_assembler_client is a modified form of the periodic_snapshotter.cpp which is included in the /examples folder of laser_assembler which is hosted at:
https://github.com/ros-perception/laser_assembler
##
