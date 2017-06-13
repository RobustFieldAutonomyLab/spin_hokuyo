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

/* Author: Vijay Pradeep */

#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_assembler/AssembleScans.h"
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

using namespace ros;
using namespace sensor_msgs;
using namespace std;

static const string SERVICE_NAME = "assemble_scans";

class TestAssembler : public testing::Test
{
public:

  NodeHandle n_;

  void SetUp()
  {
    ROS_INFO("Waiting for service [%s]", SERVICE_NAME.c_str());
    ros::service::waitForService(SERVICE_NAME);
    ROS_INFO("Service [%s] detected", SERVICE_NAME.c_str());
    received_something_ = false;
    got_cloud_ = false;
    scan_sub_  = n_.subscribe("dummy_scan",  10, &TestAssembler::ScanCallback,  (TestAssembler*)this);
  }

  void ScanCallback(const LaserScanConstPtr& scan_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!received_something_)
    {
      // Store the time of this first scan. Will be needed when we make the service call
      start_time_ = scan_msg->header.stamp;
      received_something_ = true;
    }
    else
    {
      // Make the call to get a point cloud
      laser_assembler::AssembleScans assemble_scans;
      assemble_scans.request.begin = start_time_;
      assemble_scans.request.end = scan_msg->header.stamp;
      EXPECT_TRUE(ros::service::call(SERVICE_NAME, assemble_scans));
      if(assemble_scans.response.cloud.points.size() > 0)
      {
        ROS_INFO("Got a cloud with [%u] points. Saving the cloud", (uint32_t)(assemble_scans.response.cloud.points.size()));
        cloud_ = assemble_scans.response.cloud;
        got_cloud_ = true;
        cloud_condition_.notify_all();
      }
      else
        ROS_INFO("Got an empty cloud. Going to try again on the next scan");
    }
  }

protected:
  ros::Subscriber scan_sub_;
  bool received_something_;
  ros::Time start_time_;
  bool got_cloud_;
  sensor_msgs::PointCloud cloud_;
  boost::mutex mutex_;
  boost::condition cloud_condition_;
};


void spinThread()
{
  ros::spin();
}

// Check to make sure we can get a point cloud with at least 1 point in it
TEST_F(TestAssembler, non_zero_cloud_test)
{
  // Wait until we get laser scans
  boost::mutex::scoped_lock lock(mutex_);

  while(n_.ok() && !got_cloud_)
  {
    cloud_condition_.timed_wait(lock, boost::posix_time::milliseconds(1000.0f));
  }

  EXPECT_LT((unsigned int) 0, cloud_.points.size());

  SUCCEED();
}

int main(int argc, char** argv)
{
  printf("******* Starting application *********\n");

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_assembler_node");

  boost::thread spin_thread(spinThread);

  int result = RUN_ALL_TESTS();

  ros::shutdown();

  spin_thread.join();
  return result;
}
