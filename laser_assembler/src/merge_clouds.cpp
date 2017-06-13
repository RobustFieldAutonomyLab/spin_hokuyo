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

/** \author Ioan Sucan */

/**

@b MergeClouds is a node capable of combining point clouds,
potentially from different sensors

 **/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <message_filters/subscriber.h>

class MergeClouds
{
public:

  MergeClouds(void) :
    sub1_(nh_, "cloud_in1", 1),
    sub2_(nh_, "cloud_in1", 1)
  {
    cloudOut_ = nh_.advertise<sensor_msgs::PointCloud>("cloud_out", 1);
    nh_.param<std::string>("~output_frame", output_frame_, std::string());
    nh_.param<double>("~max_frequency", max_freq_, 0.0);
    newCloud1_ = newCloud2_ = false;

    if (output_frame_.empty())
      ROS_ERROR("No output frame specified for merging pointclouds");

    // make sure we don't publish too fast
    if (max_freq_ > 1000.0 || max_freq_ < 0.0)
      max_freq_ = 0.0;

    if (max_freq_ > 0.0)
    {
      timer_ = nh_.createTimer(ros::Duration(1.0/max_freq_), boost::bind(&MergeClouds::onTimer, this, _1));
      haveTimer_ = true;
    }
    else
      haveTimer_ = false;

    tf_filter1_.reset(new tf::MessageFilter<sensor_msgs::PointCloud>(sub1_, tf_, output_frame_, 1));
    tf_filter2_.reset(new tf::MessageFilter<sensor_msgs::PointCloud>(sub2_, tf_, output_frame_, 1));

    tf_filter1_->registerCallback(boost::bind(&MergeClouds::receiveCloud1, this, _1));
    tf_filter2_->registerCallback(boost::bind(&MergeClouds::receiveCloud2, this, _1));
  }

  ~MergeClouds(void)
  {

  }

private:

  void onTimer(const ros::TimerEvent &e)
  {
    if (newCloud1_ && newCloud2_)
      publishClouds();
  }

  void publishClouds(void)
  {
    lock1_.lock();
    lock2_.lock();

    newCloud1_ = false;
    newCloud2_ = false;

    sensor_msgs::PointCloud out;
    if (cloud1_.header.stamp > cloud2_.header.stamp)
      out.header = cloud1_.header;
    else
      out.header = cloud2_.header;

    out.points.resize(cloud1_.points.size() + cloud2_.points.size());

    // copy points
    std::copy(cloud1_.points.begin(), cloud1_.points.end(), out.points.begin());
    std::copy(cloud2_.points.begin(), cloud2_.points.end(), out.points.begin() + cloud1_.points.size());

    // copy common channels
    for (unsigned int i = 0 ; i < cloud1_.channels.size() ; ++i)
      for (unsigned int j = 0 ; j < cloud2_.channels.size() ; ++j)
        if (cloud1_.channels[i].name == cloud2_.channels[j].name)
        {
          ROS_ASSERT(cloud1_.channels[i].values.size() == cloud1_.points.size());
          ROS_ASSERT(cloud2_.channels[j].values.size() == cloud2_.points.size());
          unsigned int oc = out.channels.size();
          out.channels.resize(oc + 1);
          out.channels[oc].name = cloud1_.channels[i].name;
          out.channels[oc].values.resize(cloud1_.channels[i].values.size() + cloud2_.channels[j].values.size());
          std::copy(cloud1_.channels[i].values.begin(), cloud1_.channels[i].values.end(), out.channels[oc].values.begin());
          std::copy(cloud2_.channels[j].values.begin(), cloud2_.channels[j].values.end(), out.channels[oc].values.begin() + cloud1_.channels[i].values.size());
          break;
        }

    lock1_.unlock();
    lock2_.unlock();

    cloudOut_.publish(out);
  }

  void receiveCloud1(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    lock1_.lock();
    processCloud(cloud, cloud1_);
    lock1_.unlock();
    newCloud1_ = true;
    if (!haveTimer_ && newCloud2_)
      publishClouds();
  }

  void receiveCloud2(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    lock2_.lock();
    processCloud(cloud, cloud2_);
    lock2_.unlock();
    newCloud2_ = true;
    if (!haveTimer_ && newCloud1_)
      publishClouds();
  }

  void processCloud(const sensor_msgs::PointCloudConstPtr &cloud, sensor_msgs::PointCloud &cloudOut)
  {
    if (output_frame_ != cloud->header.frame_id)
      tf_.transformPointCloud(output_frame_, *cloud, cloudOut);
    else
      cloudOut = *cloud;
  }

  ros::NodeHandle       nh_;
  tf::TransformListener tf_;

  ros::Timer            timer_;
  bool                  haveTimer_;

  ros::Publisher        cloudOut_;
  double                max_freq_;
  std::string           output_frame_;

  message_filters::Subscriber<sensor_msgs::PointCloud> sub1_;
  message_filters::Subscriber<sensor_msgs::PointCloud> sub2_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud> > tf_filter1_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud> > tf_filter2_;

  bool                    newCloud1_;
  bool                    newCloud2_;
  sensor_msgs::PointCloud cloud1_;
  sensor_msgs::PointCloud cloud2_;
  boost::mutex            lock1_;
  boost::mutex            lock2_;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "merge_clouds", ros::init_options::AnonymousName);

  ROS_WARN("laser_assembler/merge_clouds is deprecated. You should instead be using 3dnav_pr2/merge_clouds");

  MergeClouds mc;
  ros::spin();

  return 0;
}
