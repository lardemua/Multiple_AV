/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************

/**
\file  local_path_planning.h
\brief Global include file with some classes to handle data
\author Ricardo Silva
\date   June, 2018
*/

/*---General Includes---*/
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

/**
@brief Converts a point from cartezian coordinates to spheric coordinates
@param[in] Point in cartezian coordinates
@return geometry_msgs::Point Point in spheric coordinates
*/
geometry_msgs::Point xyzTortp(geometry_msgs::Point point)
{
  double X = point.x;
  double Y = point.y;
  double Z = point.z;

  double radius = sqrt((double)(double)pow(X, 2) + (double)pow(Y, 2) + (double)pow(Z, 2));
  double theta = atan2(Y, X);
  double phi = acos((double)(Z / radius));

  if (theta < 0)
    theta += M_PI * 2;

  geometry_msgs::Point point_s;

  point_s.x = radius;
  point_s.y = theta;
  point_s.z = phi;

  return point_s;
}

/**
@brief Converts a point from spheric coordinates to cartezian coordinates
@param[in] Point in spheric coordinates
@return geometry_msgs::Point Point in cartezian coordinates
*/
geometry_msgs::Point rtpToxyz(geometry_msgs::Point point)
{
  double radius = point.x;
  double theta = point.y;
  double phi = point.z;

  double X = (double)((double)cos(theta) * (double)sin(phi) * (double)radius);
  double Y = (double)((double)sin(theta) * (double)sin(phi) * (double)radius);
  double Z = (double)((double)cos(phi) * (double)radius);

  geometry_msgs::Point point_s;

  point_s.x = X;
  point_s.y = Y;
  point_s.z = Z;

  return point_s;
}
