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

//! \author Vijay Pradeep

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "message_filters/subscriber.h"

#include <deque>

// Service
#include "laser_assembler/AssembleScans.h"
#include "laser_assembler/AssembleScans2.h"

#include "boost/thread.hpp"
#include "math.h"

namespace laser_assembler
{

/**
 * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
 */
template<class T>
class BaseAssembler
{
public:
  BaseAssembler(const std::string& max_size_param_name);
  ~BaseAssembler() ;

  /**
   * \brief Tells the assembler to start listening to input data
   * It is possible that a derived class needs to initialize and configure
   * various things before actually beginning to process scans. Calling start
   * create subcribes to the input stream, thus allowing the scanCallback and
   * ConvertToCloud to be called. Start should be called only once.
   */
  void start(const std::string& in_topic_name);
  void start();


  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * BaseAssembler, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void ConvertToCloud(const std::string& fixed_frame_id, const T& scan_in, sensor_msgs::PointCloud& cloud_out) = 0 ;

protected:
  tf::TransformListener* tf_ ;
  tf::MessageFilter<T>* tf_filter_;

  ros::NodeHandle private_ns_;
  ros::NodeHandle n_;

private:
  // ROS Input/Ouptut Handling
  ros::ServiceServer build_cloud_server_;
  ros::ServiceServer assemble_scans_server_;
  ros::ServiceServer build_cloud_server2_;
  ros::ServiceServer assemble_scans_server2_;
  message_filters::Subscriber<T> scan_sub_;
  message_filters::Connection tf_filter_connection_;

  //! \brief Callback function for every time we receive a new scan
  //void scansCallback(const tf::MessageNotifier<T>::MessagePtr& scan_ptr, const T& testA)
  virtual void msgCallback(const boost::shared_ptr<const T>& scan_ptr) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp) ;
  bool assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp) ;
  bool buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
  bool assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;

  //! \brief Stores history of scans
  std::deque<sensor_msgs::PointCloud> scan_hist_ ;
  boost::mutex scan_hist_mutex_ ;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ ;

  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_ ;

  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ ;

} ;

template <class T>
BaseAssembler<T>::BaseAssembler(const std::string& max_size_param_name) : private_ns_("~")
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs ;
  private_ns_.param("tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
  if (tf_cache_time_secs < 0)
    ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

  tf_ = new tf::TransformListener(n_, ros::Duration(tf_cache_time_secs));
  ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

  // ***** Set max_scans *****
  const int default_max_scans = 400 ;
  int tmp_max_scans ;
  private_ns_.param(max_size_param_name, tmp_max_scans, default_max_scans);
  if (tmp_max_scans < 0)
  {
    ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
    tmp_max_scans = default_max_scans ;
  }
  max_scans_ = tmp_max_scans ;
  ROS_INFO("Max Scans in History: %u", max_scans_) ;
  total_pts_ = 0 ;    // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  private_ns_.param("fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME"));
  ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    ROS_ERROR("Need to set parameter fixed_frame") ;

  // ***** Set downsample_factor *****
  int tmp_downsample_factor ;
  private_ns_.param("downsample_factor", tmp_downsample_factor, 1);
  if (tmp_downsample_factor < 1)
  {
    ROS_ERROR("Parameter downsample_factor<1: %i", tmp_downsample_factor) ;
    tmp_downsample_factor = 1 ;
  }
  downsample_factor_ = tmp_downsample_factor ;
  if (downsample_factor_ != 1)
    ROS_WARN("Downsample set to [%u]. Note that this is an unreleased/unstable feature", downsample_factor_);

  // ***** Start Services *****
  build_cloud_server_    = n_.advertiseService("build_cloud",    &BaseAssembler<T>::buildCloud,    this);
  assemble_scans_server_ = n_.advertiseService("assemble_scans", &BaseAssembler<T>::assembleScans, this);
  build_cloud_server2_    = n_.advertiseService("build_cloud2",    &BaseAssembler<T>::buildCloud2,    this);
  assemble_scans_server2_ = n_.advertiseService("assemble_scans2", &BaseAssembler<T>::assembleScans2, this);

  // ***** Start Listening to Data *****
  // (Well, don't start listening just yet. Keep this as null until we actually start listening, when start() is called)
  tf_filter_ = NULL;

}

template <class T>
void BaseAssembler<T>::start(const std::string& in_topic_name)
{
  ROS_DEBUG("Called start(string). Starting to listen on message_filter::Subscriber the input stream");
  if (tf_filter_)
    ROS_ERROR("assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {
    scan_sub_.subscribe(n_, in_topic_name, 10);
    tf_filter_ = new tf::MessageFilter<T>(scan_sub_, *tf_, fixed_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&BaseAssembler<T>::msgCallback, this, _1) );
  }
}

template <class T>
void BaseAssembler<T>::start()
{
  ROS_DEBUG("Called start(). Starting tf::MessageFilter, but not initializing Subscriber");
  if (tf_filter_)
    ROS_ERROR("assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {
    scan_sub_.subscribe(n_, "bogus", 10);
    tf_filter_ = new tf::MessageFilter<T>(scan_sub_, *tf_, fixed_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&BaseAssembler<T>::msgCallback, this, _1) );
  }
}

template <class T>
BaseAssembler<T>::~BaseAssembler()
{
  if (tf_filter_)
    delete tf_filter_;

  delete tf_ ;
}

template <class T>
void BaseAssembler<T>::msgCallback(const boost::shared_ptr<const T>& scan_ptr)
{
  ROS_DEBUG("starting msgCallback");
  const T scan = *scan_ptr ;

  sensor_msgs::PointCloud cur_cloud ;

  // Convert the scan data into a universally known datatype: PointCloud
  try
  {
    ConvertToCloud(fixed_frame_, scan, cur_cloud) ;              // Convert scan into a point cloud
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }

  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock() ;
  if (scan_hist_.size() == max_scans_)                           // Is our deque full?
  {
    total_pts_ -= scan_hist_.front().points.size () ;            // We're removing an elem, so this reduces our total point count
    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  scan_hist_.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
  total_pts_ += cur_cloud.points.size () ;                       // Add the new scan to the running total of points

  //printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_) ;

  scan_hist_mutex_.unlock() ;
  ROS_DEBUG("done with msgCallback");
}

template <class T>
bool BaseAssembler<T>::buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp)
{
  ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
  return assembleScans(req, resp);
}


template <class T>
bool BaseAssembler<T>::assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp)
{
  //printf("Starting Service Request\n") ;

  scan_hist_mutex_.lock() ;
  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
  {
    i++ ;
  }
  unsigned int start_index = i ;

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].points.size ()+downsample_factor_-1)/downsample_factor_ ;
    i += downsample_factor_ ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.cloud.header.frame_id = fixed_frame_ ;
    resp.cloud.header.stamp = req.end ;
    resp.cloud.points.resize (0) ;
    resp.cloud.channels.resize (0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp.cloud.points.resize (req_pts);
    const unsigned int num_channels = scan_hist_[start_index].channels.size ();
    resp.cloud.channels.resize (num_channels) ;
    for (i = 0; i<num_channels; i++)
    {
      resp.cloud.channels[i].name = scan_hist_[start_index].channels[i].name ;
      resp.cloud.channels[i].values.resize (req_pts) ;
    }
    //resp.cloud.header.stamp = req.end ;
    resp.cloud.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {

      // Sanity check: Each channel should be the same length as the points vector
      for (unsigned int chan_ind = 0; chan_ind < scan_hist_[i].channels.size(); chan_ind++)
      {
        if (scan_hist_[i].points.size () != scan_hist_[i].channels[chan_ind].values.size())
          ROS_FATAL("Trying to add a malformed point cloud. Cloud has %u points, but channel %u has %u elems", (int)scan_hist_[i].points.size (), chan_ind, (int)scan_hist_[i].channels[chan_ind].values.size ());
      }

      for(unsigned int j=0; j<scan_hist_[i].points.size (); j+=downsample_factor_)
      {
        resp.cloud.points[cloud_count].x = scan_hist_[i].points[j].x ;
        resp.cloud.points[cloud_count].y = scan_hist_[i].points[j].y ;
        resp.cloud.points[cloud_count].z = scan_hist_[i].points[j].z ;

        for (unsigned int k=0; k<num_channels; k++)
          resp.cloud.channels[k].values[cloud_count] = scan_hist_[i].channels[k].values[j] ;

        cloud_count++ ;
      }
      resp.cloud.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, scan_hist_.size(), (int)resp.cloud.points.size ()) ;
  return true ;
}

template <class T>
bool BaseAssembler<T>::buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{
  ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
  return assembleScans2(req, resp);
}

template <class T>
bool BaseAssembler<T>::assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{
  AssembleScans::Request tmp_req;
  AssembleScans::Response tmp_res;
  tmp_req.begin = req.begin;
  tmp_req.end = req.end;
  bool ret = assembleScans(tmp_req, tmp_res);

  if ( ret )
  {
    sensor_msgs::convertPointCloudToPointCloud2(tmp_res.cloud, resp.cloud);
  }
  return ret;
}
}
