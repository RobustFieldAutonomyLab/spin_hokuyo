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
float max_z;
int z_obtained = 0;
float inf = numeric_limits<float>::infinity();

void obtainZ (const std_msgs::Float64 msg)
{
    max_z = msg.data;
    z_obtained = 1;
}

void filterGround (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    while(z_obtained == 0)
    {
       ros::spinOnce();
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    pcl::PointCloud<pcl::PointXYZ> aux;

    aux.header = cloud.header;
    aux.height = cloud.height;
    aux.width = cloud.width;
 
   int n = cloud.width;
ROS_ERROR_STREAM(n);

    aux.points.resize(201585);

    for (int i=1; i <= n; i++)
    {
       if(cloud.points[i].z > max_z)
       {
          aux.points[i].x = cloud.points[i].x;
          aux.points[i].y = cloud.points[i].y;
          aux.points[i].z = cloud.points[i].z;
       }

       else
       {
          aux.points[i].x = inf;
          aux.points[i].y = inf;
       }
    }
ROS_WARN_STREAM("hi");
    sensor_msgs::PointCloud2 filtered;
    pcl::toROSMsg(aux, filtered);
    pub.publish(filtered);
    z_obtained = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hokuyo_ground_filter");
    ros::NodeHandle nh;
    ros::Subscriber sub_1=nh.subscribe<std_msgs::Float64>("/max_z", 1, &obtainZ);
    ros::Subscriber sub_2=nh.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud", 1, &filterGround);
    pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_test", 1);
    ros::spin();
    return 0;
}
