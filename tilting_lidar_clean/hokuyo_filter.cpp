#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

using namespace std;
float inf = numeric_limits<float>::infinity();
ros::Publisher pub;

void filter(const sensor_msgs::LaserScan msg)
{
   sensor_msgs::LaserScan aux;
   aux.header = msg.header;
   aux.angle_min = msg.angle_min;
   aux.angle_max = msg.angle_max;
   aux.angle_increment = msg.angle_increment;
   aux.scan_time = msg.scan_time;
   aux.range_min = 0;
   aux.range_max = inf;
   aux.intensities[1080];
   aux.ranges.resize(1080);
   for(int n = 0; n < 1081; n++)
   {
      if (msg.ranges[n] > 0.3)
      {
ROS_WARN_STREAM("good pt");
         aux.ranges[n] = msg.ranges[n];
      }
 
      else
      {
ROS_ERROR_STREAM("bad pt");
         aux.ranges[n] = inf;
      }
   }

   pub.publish(aux);
   ROS_INFO_STREAM("Published Filtered");
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "hokuyo_filter");
   ros::NodeHandle nh;

   ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, filter);

   pub = nh.advertise<sensor_msgs::LaserScan>("hokuyo_filtered", 1);

   ros::spin();

}
