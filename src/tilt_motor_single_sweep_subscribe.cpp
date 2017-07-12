#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<dynamixel_msgs/JointState.h>
#include<cmath>
#include<laser_assembler/AssembleScans.h>
#include<ros/time.h>
#include<std_msgs/Time.h>
#include<std_msgs/Empty.h>

/*This code allows the motor to move back and forth to a maximum and minimum number of degrees defined in parameters.
  This iteration performs initializes the servo and then waits for messages through and empty message to perform sweeps.
  This version also sends times to cloud_compiler_subscriber to assemble point clouds.*/

using namespace std;

//global variables
float error;
int max_angle;
int min_angle;
float pause_time;

//obtains error from message
void obtainValues(const dynamixel_msgs::JointState &msg) {
    error = msg.error;
}

//creates all commands for the motor
class Dynamixel {
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Publisher pub_3;
    ros::Subscriber sub;

    public:
    Dynamixel();
    void checkError();
    void moveMotor(double position);
    void startTime();
    void endTime();
};

//creates publishers and subscriber
Dynamixel::Dynamixel() {
    //publishe motor movements
    pub_1 = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10);
    //publish start time of sweep
    pub_2 = nh.advertise<std_msgs::Time>("/time/start_time", 1);
    //publish end time of sweep
    pub_3 = nh.advertise<std_msgs::Time>("/time/end_time", 1);
    //subscribe to /state for motor error
    sub   = nh.subscribe("/tilt_controller/state", 1, &obtainValues);
}

//creates message and publishes -> degree to radian to publish
void Dynamixel::moveMotor(double position) {
    double convert = (position * 3.14/180);
    std_msgs::Float64 aux;
    aux.data = convert;
    pub_1.publish(aux);
    ROS_INFO_STREAM(aux);
}

//ensures proper alignment
void Dynamixel::checkError() {
    ros::spinOnce(); //get starting value of motor position

    while((abs (error))>0.05) {  //keep waiting and checking error until < 0.05
        ros::Duration(.1).sleep();
        ros::spinOnce();
    }    
}

//publishes start time for cloud compiler
void Dynamixel::startTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_2.publish(msg);
}

//publishes end time for cloud compiler
void Dynamixel::endTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_3.publish(msg);
}

//initilazies motor to min angle
void initialize() {
    Dynamixel motor_1;

    motor_1.moveMotor(min_angle);
    ros::Duration(pause_time).sleep();
    motor_1.checkError();
    ros::Duration(pause_time).sleep();
}

//performs one sweep min -> max -> min
void sweep(const std_msgs::Empty &msg) {
    Dynamixel motor;

    motor.startTime();
    motor.moveMotor(max_angle);
    ros::Duration(pause_time).sleep();
    motor.checkError();
    ros::Duration(pause_time).sleep();

    motor.moveMotor(min_angle);
    ros::Duration(pause_time).sleep();
    motor.checkError();
    motor.endTime();
    ros::Duration(pause_time).sleep();
    ROS_INFO("Finished One Sweep!");
}

//main
int main(int argc, char **argv) {

    //initialize
    ros::init(argc, argv, "Motor_Tilt");
    ros::NodeHandle nh;

    //variables
    int max;
    int min;
    double pause;
    Dynamixel motor;

    //get parameters
    nh.param("maximum", max, 92);
    nh.param("minimum", min, -92);
    nh.param("pause", pause, 0.1);

    //transfer parameters to global variables
    max_angle = max;
    min_angle = min;
    pause_time = pause;

    //Wait for servo init by waiting for /state topic
    ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(100));

    //subscribe to empty message to run sweep
    ros::Subscriber sub=nh.subscribe("/perform_sweep", 1, &sweep);
    
    //pause to allow motor object to initialize and set to min_angle
    ros::Duration(1).sleep();
    initialize(); 
    
    //wait for empty messages to perform sweep
    ros::spin();
}
