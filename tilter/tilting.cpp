#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<tf/transform_broadcaster.h>

/*This code allows the motor to move a certain interval, specified in degrees, and then pause.  A maximum and minimum angle can be specified, as well as the starting point.  The pause and interval ammount can be changed as well.

In this version, the motor first goes to the starting position, then the maximum angle, and finally the minumum angle.  The motor than oscillates between the maximum and minimum angles. */

using namespace std;

float current;
float future;

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

void transform()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(future *3.14/180, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser"));
}
  
int main(int argc, char **argv)
{
  //setup
  ros::init(argc, argv, "Test_Tilting");
  ros::NodeHandle nh;
  Publisher motor;

  //variables
  int start;
  int interval;
  int max;
  int min;
  double pause;
  float go = 0;

  //intitialize parameters
  nh.param("start", start, 0);
  nh.param("interval", interval, 45);
  nh.param("maximum", max, 90);
  nh.param("minimum", min, -90);
  nh.param("pause", pause, 5.0);
  current = start;


while(ros::ok())
  {
    while(current < max)
    {
      if(go == 0)
      {
        future = start;
        go = 1;
      }
      else
      {
        future = current + interval; 
      }

      motor.move(future);
      current = future;
      ros::Duration(pause).sleep();
      transform();
    }
    current = max; 

    while (current > min)
    {
      future = current - interval; 
      motor.move(future);
      current = future;
      ros::Duration(pause).sleep();
      transform();
    }
    current = min;

  }
}
