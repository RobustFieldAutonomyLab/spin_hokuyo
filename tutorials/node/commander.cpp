#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<stdio.h>
#include<dynamixel_msgs/JointState.h>
#include<cmath>

/*  This program takes in numbers from the /commands topic and prepares them before publishing to the servo.  The number
on the /commands topic corresponds to a location on the servo's rotation in degrees.  Each position is fixed
(i.e. There is a 0 deg. position and a 180 deg. position.  Sending a command of 180 will not necessarily make the motor
turn 180 deg.).  The program then converts the degree value into radians and publishes the value to the servo.  The program
also allows for enough time to pass prior to the next command to allow the servo to move */

using namespace std;

//global variables
float error = 1;
int go = 0;
float place;

//obtains error from message
void obtainValues(const dynamixel_msgs::JointState &msg)
{
  error = msg.error;
}  

//creates all commands for each motor
class Dynamixel
{
  private:
  ros::NodeHandle nh;
  ros::Publisher pub_n;
  public:
  Dynamixel();
  void checkError();
  void moveMotor(double position);
};

//creates publisher
Dynamixel::Dynamixel()
{
  pub_n = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10);
}

//creates message and publishes -> degree to radian to publish
 void Dynamixel::moveMotor(double position)
{
  double convert = (position * 3.14/180);
  std_msgs::Float64 aux;
  aux.data = convert;
  pub_n.publish(aux);
  //ROS_INFO_STREAM(aux);
}

//ensures proper alignment
void Dynamixel::checkError()
{
  ros::spinOnce();
  ROS_ERROR_STREAM(error);
  while(abs(error)>0.02)
  {
    ROS_INFO_STREAM("hi");
    ros::Duration(0.5).sleep();
    ROS_WARN_STREAM(error);
    ros::spinOnce();

  }
}

//obtains requested position
void transfer(const std_msgs::Float64 &msg)
{
   place = msg.data;
   go = 1;
}

//main
int main (int argc, char **argv)
{
  //initializes
  ros::init(argc, argv, "commander");
  ros::NodeHandle nh;
  
  //creates 1 Dynamixel named motor
  Dynamixel motor;

  //subscribes to external requests
  ros::Subscriber sub_2=nh.subscribe("/commands", 1, &transfer);  //external

  ros::Subscriber sub=nh.subscribe("/tilt_controller/state", 5, &obtainValues); //checks error

  //Wait for servo init by waiting for /state message
  ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(100));
  
  while(ros::ok())
  {
  //if new request has been received
  if(go==1)
  {
    //move motor to request
    motor.moveMotor(place);
    ros::Duration(0.5).sleep();
    motor.checkError();
    ros::Duration(0.5).sleep(); //saftety break
    go = 0;
  }
  
  else
  { 
    ros::spinOnce();
  }
  }
}
