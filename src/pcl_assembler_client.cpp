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
#include<ros/time.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include"dynamixel_msgs/JointState.h"
#include"std_msgs/Time.h"

/* This is a modified periodic_snapshotter.cpp file that acts both as a timer and subscriber, depending on the value of the
assembled_cloud_mode parameter.  It defaults to subscriber.  The node calls a service from the point_cloud2_assembler node
from the laser_assembler package to create the compiled point cloud and then publishes the result.
Timer:
As a timer, the node performs its function every 5 seconds.  To use this function, the assembled_cloud_mode parameter must
be set to time.
Subscriber:
As a subscriber, the node subscribes to start and end times published while tilting the motor and sends those as necessary
to the compilation service to put all of the point clouds together.  One cloud is published for each complete sweep
(i.e. -90 -> +90 -> -90).  This is the default setting of the node.*/

using namespace laser_assembler;

//global variables
ros::Time start;
ros::Time end;
ros::Time init;
int go = 0;
std::string assembled_cloud_mode;
double scan_time;

//call back for start time, saves in global variable
void startTime(const std_msgs::Time &msg) {
    start = msg.data;
}

//callback for end time, saves in global variable and updates go to start compilation
void endTime(const std_msgs::Time &msg) {
    end = msg.data;
    go = 1;
}

//compilation class created by combine_clouds with modifications to remove timer and work with motor times
class PeriodicSnapshotter {

    public:

    PeriodicSnapshotter() {
        // Create a publisher for the clouds that we assemble
        pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);

        // Create the service client for calling the assembler
        client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

        // Start the timer that will trigger the processing loop (timerCallback)
        timer_ = n_.createTimer(ros::Duration(scan_time), &PeriodicSnapshotter::timerCallback, this);

        // Need to track if we've called the timerCallback at least once
        first_time_ = true;
    }

    void compile() {
        // Populate our service request based on motor times
        AssembleScans2 srv;
        srv.request.begin = start;
        srv.request.end   = end;

        // Make the service call
        if (client_.call(srv)) {
            //ROS_INFO_STREAM("Published Cloud") ;
            pub_.publish(srv.response.cloud);
        }
        else {
            //ROS_ERROR("Error making service call\n") ;
        } 
    }

    void timerCallback(const ros::TimerEvent& e) {
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
        if (first_time_) {
            first_time_ = false;

        // Populate our service request based on our timer callback times
        AssembleScans2 srv;
        srv.request.begin = init;
        srv.request.end   = e.current_real;

        // Make the service call
        if (client_.call(srv)) {
            //ROS_INFO("Published Cloud");
            pub_.publish(srv.response.cloud);
        }

        else {
            //ROS_ERROR("Error making service call\n") ;
        }

            return;
        }

        // Populate our service request based on our timer callback times
        AssembleScans2 srv;
        srv.request.begin = e.last_real;
        srv.request.end   = e.current_real;

        // Make the service call
        if (client_.call(srv)) {
            //ROS_INFO("Published Cloud") ;
            pub_.publish(srv.response.cloud);
        }  

        else {
            //ROS_ERROR("Error making service call\n") ;
        }

        //ROS_ERROR_STREAM(scan_time);
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::ServiceClient client_;
    ros::Timer timer_;
    bool first_time_;
} ;

//main
int main(int argc, char **argv)
{
    //initialize and wait for necessary services, etc.
    ros::init(argc, argv, "Cloud_Compiler");
    ros::NodeHandle n;
    //ROS_INFO("Waiting for [build_cloud] to be advertised");
    
    //parameter for timer vs. subscriber and length of timer
    n.param<std::string>("assembled_cloud_mode", assembled_cloud_mode, "subscriber");
    n.param<double>("scan_time", scan_time, 5);

    //Wait for build cloud service to init
    ros::service::waitForService("build_cloud");
    //ROS_INFO_STREAM("Found build_cloud! Starting the Cloud Compiler");

    //SUBSCRIBER intialization
    if (assembled_cloud_mode == "subscriber")
    {
        //Wait for dynamixel servo to init by waiting for /state topic
        ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(20));   
        
        //subscribes to start and end time published by tilting motor
        ros::Subscriber sub_1=n.subscribe("/time/start_time", 1, &startTime);
        ros::Subscriber sub_2=n.subscribe("/time/end_time", 1, &endTime);
    }

    PeriodicSnapshotter snapshotter;
    init = ros::Time::now();
    
    //SUBSCRIBER MAIN LOOP
    if (assembled_cloud_mode == "subscriber")
    {
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
    }

    //TIMER main loop
    else if (assembled_cloud_mode == "time")
    {
        //wait for timer to begin service call
        ros::spin();
    }

return 0;

}
