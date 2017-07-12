#include<tf/transform_broadcaster.h>
#include<ros/ros.h>
#include<dynamixel_msgs/JointState.h>

/* This node publishes the tf between the laser scan and the servo.  This is based on the angle published by the servo. */

//Module that applies transform to laser scan of tilting hokuyo laser
using namespace std;

//global variables
float pos;

//Recieves position values from dynamixel servo and uses angle to apply transform to laser scan
void obtainValues(const dynamixel_msgs::JointState &msg) 
{
    //gets position from message
    pos = msg.current_pos;
    
    //perform transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(pos, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "servo", "laser"));
}

//main
int main(int argc, char **argv) 
{
    //initialize
    ros::init(argc, argv, "tilt_transform");
    ros::NodeHandle nh;
  
    //subscirber to current position
    ros::Subscriber position_sub = nh.subscribe("/tilt_controller/state", 5, &obtainValues);

    //wait for updates in position
    ros::spin();
}
