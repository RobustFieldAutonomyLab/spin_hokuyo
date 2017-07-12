#include<ros/ros.h>
#include<std_msgs/Float64.h>

/* This code was based on the examples provided in Chap. 8 of "Effective Robotics Programming with ROS - Third Edition"
by Anil Mahtani, Luis Sanchez, Enrique Fernandea, and Aaron Martinez.
This is a shell for publishing servo movement to the /commands topic.  The /commands published here are taken into the
commander node to be processed and then transmitted to the servo.  The basic_motors.launch file must be used prior
to running any nodes created with this code.  Alternatively, this node can be included in a new launch file (copied and
then modified from basic_motors.launch) and will work.

This shell creates a Pulbisher class to which motor commands can be prepared and then published to the /commands topic.
To use this publisher, use motor.move("int") in the main function.  Replace "int" with the value, in degrees, to which you
want the motor to move.  Please see commander.cpp for additional information on movement.  */

using namespace std;

class Publisher
{
  private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  public:
  Publisher();
  void move(float request);
};

Publisher::Publisher()
{
  pub=nh.advertise<std_msgs::Float64>("commands", 1000);
}

void Publisher::move(float request)
{
  std_msgs::Float64 msg;
  msg.data = request;
  pub.publish(msg);
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Shell");
  ros::NodeHandle nh;
  Publisher motor;

}
