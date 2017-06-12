#include<tf/transform_broadcaster.h>
#include<ros/ros.h>
#include<dynamixel_msgs/JointState.h>

//Module that applies transform to laser scan of tilting hokuyo laser

using namespace std;
float pos;

void obtainValues(const dynamixel_msgs::JointState &msg) {
    pos = msg.current_pos;
}

void transform() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(pos, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "laser"));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "tilt_transform");
    ros::NodeHandle nh;
  
    ros::Subscriber position_sub = nh.subscribe("/tilt_controller/state", 5, &obtainValues);
  
    while(ros::ok()) {
        transform();
        ros::spinOnce();
    }
}
