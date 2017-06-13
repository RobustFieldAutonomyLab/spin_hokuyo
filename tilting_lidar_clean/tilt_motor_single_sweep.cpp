#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<dynamixel_msgs/JointState.h>
#include<cmath>
#include<laser_assembler/AssembleScans.h>
#include<ros/time.h>
#include<std_msgs/Time.h>
/*This code allows the motor to move back and forth to a maximum and minimum number of degrees defined in parameters.*/

using namespace std;

float error;

//obtains error from message
void obtainValues(const dynamixel_msgs::JointState &msg) {
    error = msg.error;
}

//creates all commands for each motor
class Dynamixel {
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_n;
    public:
    Dynamixel();
    void checkError();
    void moveMotor(double position);
};

//creates publisher
Dynamixel::Dynamixel() {
    pub_n = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10);
}

//creates message and publishes -> degree to radian to publish
void Dynamixel::moveMotor(double position) {
    double convert = (position * 3.14/180);
    std_msgs::Float64 aux;
    aux.data = convert;
    pub_n.publish(aux);
    ROS_INFO_STREAM(aux);
}

//ensures proper alignment
void Dynamixel::checkError() {
    ros::spinOnce();
    while((abs (error))>0.05) {
     ros::Duration(.1).sleep();
     ros::spinOnce();
    }
}

class startTime
{
  private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  
  public:
  startTime();
  void send();
};

startTime::startTime()
{
  pub=nh.advertise<std_msgs::Time>("/time/start_time", 1);
}

void startTime::send()
{
  std_msgs::Time msg;
  msg.data = ros::Time::now();
  pub.publish(msg);
}

class endTime
{
  private:
  ros::NodeHandle nh;
  ros::Publisher pub;

  public:
  endTime();
  void send();
};

endTime::endTime()
{
  pub=nh.advertise<std_msgs::Time>("/time/end_time", 1);
}

void endTime::send()
{
  std_msgs::Time msg;
  msg.data = ros::Time::now();
  pub.publish(msg);
}


using namespace laser_assembler;

int main(int argc, char **argv) {

    ros::init(argc, argv, "Motor_Tilt");
    ros::NodeHandle nh;

    Dynamixel motor;
    startTime start;
    endTime end;

    //variables
    int max;
    int min;
    double pause;

    //initialize subscription
    ros::Subscriber sub=nh.subscribe("/tilt_controller/state", 1, &obtainValues); //checks error

    //intitialize parameters
    nh.param("maximum", max, 92);
    nh.param("minimum", min, -92);
    nh.param("pause", pause, 0.03);

    //Wait for servo init
    ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(20));

    motor.moveMotor(min);
    ros::Duration(pause).sleep();
    motor.checkError();
    ros::Duration(pause).sleep();


    while(ros::ok()) {

        start.send();
        motor.moveMotor(max);
	ros::Duration(pause).sleep();
        motor.checkError();
        ros::Duration(pause).sleep();

        motor.moveMotor(min);
	ros::Duration(pause).sleep();
        motor.checkError();
        end.send();
        ros::Duration(pause).sleep();
    }

}
