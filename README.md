# spin_hokuyo
spinning Hokuyo form 3D point cloud<br />
dynamixel_motor drivers required to control servor<br />

# Setup Directions
1. Install Dynamixel Drivers from ROS:<br />
sudo apt-get install ros-indigo-dynamixel-motor<br />
2. Download this repocitory into your src file.<br />
3. In the motors_base/launch folder, update the dynamixel_servos.yaml to match your servo.<br />
For information on how to fill out the .yaml file, please see dynamixel_servos_basic.yaml under motors_base/launch.<br />
4. catkin_make and source the files, then:<br />
roslaunch motors_base basic_motors.launch<br />
The motor controller and spawner should properly generate topics that correspond to the motor. (i.e. no red text)<br />
For errors, please see below. <br />
5. In a new terminal, open the rostopics list.  There should be two new commands: /tilt_controller/command and /tilt_controller/state.  /command will issue new positions to the motor via Float64 (try this using rostopic pub).  /state will list various pieces of information about the motor (rostopic echo).<br />
6. If this is fully functional, kill the node and do:<br />
roslaunch tilting_lidar_clean tilting_lidar_continuous.launch<br />
This should cause the motor to regularly sweep from 90 to -90 and generate a point cloud and occupancy map.<br />
Be sure to update the dynamixel_servos_tilting.yaml for position, speed, etc.<br />

# Possible Dynamixel Errors
Is the USB properly inserted and setup for your computer?<br />
Is the Dynamixel connected to a 12V power source via the adapter?<br />
Is the switch on the USB adapter at the proper loaction?<br />
