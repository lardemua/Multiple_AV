/**************************************************************************************************
   Software License Agreement (BSD License)
   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.
   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
   FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
   IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************************/

/**
 * @file ackermann_vehicle_gazebo.cpp
 * @brief Convertsline laser scan to point cloud
 * @author Ricardo Silva
 * @version v0
 * @date 2018-06-06
 */

#include <laser_geometry/laser_geometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/**
 @brief Class to convert laser scan
*/
class lineFilter
{
public:
  lineFilter();
  void lineCallback(const sensor_msgs::LaserScan::ConstPtr& line);

private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector;
  tf::TransformListener tfListener;

  ros::Publisher pointcloudPub;
  ros::Subscriber laserlineSub;
};

/**
   @brief lineFilter class constructor
   @return void
 */
lineFilter::lineFilter()
{
  laserlineSub = node_.subscribe<sensor_msgs::LaserScan>("/line_laser_scan", 1000, &lineFilter::lineCallback, this);
  pointcloudPub = node_.advertise<sensor_msgs::PointCloud2>("/line_pcl", 1000, false);
  tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

/**
   @brief Function to convert laser scan to point cloud
   @param laser scan
   @return void
 */
void lineFilter::lineCallback(const sensor_msgs::LaserScan::ConstPtr& line)
{
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2 cloud_v;
  projector.transformLaserScanToPointCloud("base_link", *line, cloud, tfListener);
  cloud.header.frame_id = "line_link";

  if (tfListener.waitForTransform("laser_link", "line_link", ros::Time::now(), ros::Duration(1.0)))
  {
    pcl_ros::transformPointCloud("laser_link", cloud, cloud_v, tfListener);
    pointcloudPub.publish(cloud_v);
  }
}

/**
   @brief Main function to conversion
   @param argc
   @param argv
   @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cirkit_unit03_gazebo_line_node");

  lineFilter laserline;

  ros::spin();

  return 0;
}