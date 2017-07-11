#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include<std_msgs/Float64.h>

using namespace std;

ros::Publisher pub;

void pclConversion (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std_msgs::Float64 aux;

    int n = cloud.width;
    float max_z = -999;

    for (int i=1; i <= n; i++)
    {
       if (cloud.points[i].z > max_z)
       {
          max_z = cloud.points[i].z;
          aux.data = cloud.points[i].z;
       }
    }

    pub.publish(aux);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pcl_Conversion");
   ros::NodeHandle nh;
   ros::Subscriber sub=nh.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud", 1, &pclConversion);
   //typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
   //pub=nh.advertise<PCLCloud>("pcl_test", 1);

   pub = nh.advertise<std_msgs::Float64>("max_z", 1);
   ros::spin();
   return 0;
}
