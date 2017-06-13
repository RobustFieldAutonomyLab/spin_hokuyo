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


#include "laser_assembler/base_assembler_srv.h"


using namespace std;

namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser scans) and generates a point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloudAssemblerSrv : public BaseAssemblerSrv<sensor_msgs::PointCloud>
{
public:
  PointCloudAssemblerSrv()
  {

  }

  ~PointCloudAssemblerSrv()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::PointCloud& scan)
  {
    return scan.points.size() ;
  }

  void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::PointCloud& scan_in, sensor_msgs::PointCloud& cloud_out)
  {
    tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out) ;
    return ;
  }

private:

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_assembler");
  ros::NodeHandle n;
  ROS_WARN("The point_cloud_assembler_srv is deprecated. Please switch to "
           "using the laser_scan_assembler. Documentation is available at "
           "http://www.ros.org/wiki/laser_assembler");
  PointCloudAssemblerSrv pc_assembler;
  pc_assembler.start() ;
  ros::spin();

  return 0;
}
