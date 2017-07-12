#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

//This node removes points from the Hokuyo's LaserScan that are the robot
//The removal of the robot also comes from adjusting the angle viewed by the Hokuyo in the launch file

using namespace std;

//global variable equal to infinity
float inf = numeric_limits<float>::infinity();
ros::Publisher pub;

//removes points that correspond to robot by setting them equal to infinity
void filter(const sensor_msgs::LaserScan msg)
{
   sensor_msgs::LaserScan aux; //create new LaserScan msg for filtered points

   //transfer values of original scan unrelated to seeing the robot
   aux.header = msg.header;
   aux.angle_min = msg.angle_min;
   aux.angle_max = msg.angle_max;
   aux.angle_increment = msg.angle_increment;
   aux.scan_time = msg.scan_time;
   aux.range_min = 0;
   aux.range_max = inf;
   aux.intensities[920];

   //initialize the ranges array to the 920 values needed
   aux.ranges.resize(920);

   //checks each ranges[] 0 -> 920 to see if it is the robot
   //robot is defined as distances < 0.5 m
   for(int n = 0; n < 921; n++)
   {
      if (msg.ranges[n] > 0.5)
      { 
         //directly transfer ranges[] value if not robot
         aux.ranges[n] = msg.ranges[n];
      }
 
      else
      {
         //set ranges[] to inf to "remove" point if it is the robot
         aux.ranges[n] = inf;
      }
   }

   //publish filtered cloud
   pub.publish(aux);
}

//main
int main(int argc, char **argv)
{
   //initialize
   ros::init(argc, argv, "hokuyo_robot_filter");
   ros::NodeHandle nh;

   //subscribe to LaserScan messages pulished by Hokuyo
   ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, filter);

   //publisher for filteres scans
   pub = nh.advertise<sensor_msgs::LaserScan>("hokuyo_filtered", 1);

   //wait for new scans
   ros::spin();

}
