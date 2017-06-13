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


#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_assembler/base_assembler_srv.h"
#include "filters/filter_chain.h"

using namespace laser_geometry;
using namespace std ;

namespace laser_assembler
{

/**
 * \brief Maintains a history of laser scans and generates a point cloud upon request
 * \section params ROS Parameters
 * - (Several params are inherited from laser_assembler::BaseAssemblerSrv)
 * - \b "~ignore_laser_skew" (bool) - Specifies the method to project laser data
 *   - true -> Account for laser skew, and compute the transform for each laser point (This is currently really slow!)
 *   - false-> Don't account for laser skew, and use 1 transform per scanline. (This might have a little error when moving fast)
 * \section services ROS Services
 * - "~build_cloud" - Inhertited from laser_assembler::BaseAssemblerSrv
 */
class LaserScanAssemblerSrv : public BaseAssemblerSrv<sensor_msgs::LaserScan>
{
public:
  LaserScanAssemblerSrv() : filter_chain_("sensor_msgs::LaserScan")
  {
    // ***** Set Laser Projection Method *****
    private_ns_.param("ignore_laser_skew", ignore_laser_skew_, true);

    // configure the filter chain from the parameter server
    filter_chain_.configure("filters", private_ns_);
  }

  ~LaserScanAssemblerSrv()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::LaserScan& scan)
  {
    return scan.ranges.size();
  }

  void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud& cloud_out)
  {
    // apply filters on laser scan
    filter_chain_.update (scan_in, scan_filtered_);

    // convert laser scan to point cloud
    if (ignore_laser_skew_)  // Do it the fast (approximate) way
    {
      projector_.projectLaser(scan_filtered_, cloud_out);
      if (cloud_out.header.frame_id != fixed_frame_id)
        tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
    }
    else                     // Do it the slower (more accurate) way
    {
      int mask = laser_geometry::channel_option::Intensity + laser_geometry::channel_option::Distance + laser_geometry::channel_option::Index + laser_geometry::channel_option::Timestamp;
      projector_.transformLaserScanToPointCloud (fixed_frame_id, scan_filtered_, cloud_out, *tf_, mask);
    }
    return;
  }

private:
  bool ignore_laser_skew_;
  laser_geometry::LaserProjection projector_;

  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  mutable sensor_msgs::LaserScan scan_filtered_;

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_assembler");
  ros::NodeHandle n;
  ROS_WARN("The laser_scan_assembler_srv is deprecated. Please switch to "
           "using the laser_scan_assembler. Documentation is available at "
           "http://www.ros.org/wiki/laser_assembler");
  LaserScanAssemblerSrv pc_assembler;
  pc_assembler.start();
  ros::spin();

  return 0;
}
