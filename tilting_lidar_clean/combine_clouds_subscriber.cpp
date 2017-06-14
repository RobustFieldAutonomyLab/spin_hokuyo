/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

#include"dynamixel_msgs/JointState.h"
#include"std_msgs/Time.h"
#include<ros/time.h>

/* This is a modified combine_clouds file that subscribes to times published while tilting the motor and sends those as necessary
   to the compilation service to put all of the point clouds together*/

using namespace laser_assembler;

//global variables
ros::Time start;
ros::Time end;
int go = 0;

//call back for start time, saves in global variable
void startTime(const std_msgs::Time &msg)
{
  start = msg.data;
}

//callback for end time, saves in global variable and updates go to start compilation
void endTime(const std_msgs::Time &msg)
{
  end = msg.data;
  go = 1;
}

//compilation class created by combine_clouds with modifications to remove timer and work with our times
class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

  }

  void compile()
  {
    // Populate our service request based on our timer callback times
    AssembleScans2 srv;
    srv.request.begin = start;
    srv.request.end   = end;

    // Make the service call
    if (client_.call(srv))
    {
      ROS_INFO_STREAM("Published Cloud") ;
      pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
} ;


int main(int argc, char **argv)
{
  //initialize and wait for necessary services, etc.
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  
  //Wait for build cloud service to init
  ros::service::waitForService("build_cloud");
  
  //Wait for dynamixel servo to init
  ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(20));
  
  ROS_INFO_STREAM("Found build_cloud! Starting the snapshotter");
  
  //subscribes to start and end time published by tilting motor
  ros::Subscriber sub_1=n.subscribe("/time/start_time", 1, &startTime);
  ros::Subscriber sub_2=n.subscribe("/time/end_time", 1, &endTime);

  PeriodicSnapshotter snapshotter;

  while(ros::ok())
  {
    //when an end time comes in (which only occurs after start time is updated) run the compiler
    if(go==1)
    {
      snapshotter.compile();
      go = 0;
    }

    //wait for messages when not compiling
    else
    {
      ros::spinOnce();
    }
  
  //pause to save computing power
  ros::Duration(0.01).sleep();
  }
  
return 0;
}

