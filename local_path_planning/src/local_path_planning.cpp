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
   \file  local_path_planning.cpp
   \brief Algorithm for subscribing to lidar data and compute the free space
   \author Ricardo Silva
   \date   June, 2018
 */
#include "local_path_planning/local_path_planning.h"

namespace polygon_analise
{
class polygonAnalise
{
public:
  polygonAnalise(string topicName, string frame_id);

  void polygonDataTreatment(const geometry_msgs::PolygonStamped::ConstPtr& polygon);

  geometry_msgs::PointStamped compute2DPolygonCentroid(const geometry_msgs::PolygonStamped& polygon_in);

  geometry_msgs::PointStamped getPolygonCentroid();

private:
  ros::NodeHandle n;

  ros::Subscriber polygonSub;
  ros::Publisher centroidPub;

  string topicName;
  string frameId;

  geometry_msgs::PointStamped centroid;
};

polygonAnalise::polygonAnalise(string topicName, string frame_id)
{
  this->topicName = topicName;
  this->frameId = frame_id;

  polygonSub = n.subscribe(topicName, 1000, &polygonAnalise::polygonDataTreatment, this);
  centroidPub = n.advertise<geometry_msgs::PointStamped>("polygon_centroid", 1000);
  ROS_INFO("Topic %s subscribed!", topicName.c_str());
}

void polygonAnalise::polygonDataTreatment(const geometry_msgs::PolygonStamped::ConstPtr& polygon)
{
  centroid = compute2DPolygonCentroid(*polygon);

  /*---Publisher for the Polygon---*/
  centroidPub.publish(centroid);
}

geometry_msgs::PointStamped polygonAnalise::compute2DPolygonCentroid(const geometry_msgs::PolygonStamped& polygon_in)
{
  // geometry_msgs::Point32 centroid;
  geometry_msgs::PointStamped centroid;
  centroid.header = polygon_in.header;
  geometry_msgs::Point32 p0;
  geometry_msgs::Point32 p1;
  centroid.point.x = 0;
  centroid.point.y = 0;
  centroid.point.z = 0;
  p0.x = 0.0;  // Current vertex X
  p0.y = 0.0;  // Current vertex Y
  p0.z = 0.0;
  p1.x = 0.0;  // Next vertex X
  p1.y = 0.0;  // Next vertex Y
  p1.z = 0.0;
  double signedArea = 0.0;
  double a = 0.0;  // Partial signed area
  float xx = polygon_in.polygon.points[0].x;
  int vertexCount = polygon_in.polygon.points.size();

  // For all vertices except last
  int i = 0;
  for (i = 0; i < vertexCount - 1; ++i)
  {
    p0 = polygon_in.polygon.points.at(i);
    p1 = polygon_in.polygon.points.at(i + 1);
    a = p0.x * p1.y - p1.x * p0.y;
    signedArea += a;
    centroid.point.x += (p0.x + p1.x) * a;
    centroid.point.y += (p0.y + p1.y) * a;
  }

  // Do last vertex separately to avoid performing an expensive
  // modulus operation in each iteration.
  p0 = polygon_in.polygon.points.at(i);
  p1 = polygon_in.polygon.points.at(0);
  a = p0.x * p1.y - p1.x * p0.y;
  signedArea += a;
  centroid.point.x += (p0.x + p1.x) * a;
  centroid.point.y += (p0.y + p1.y) * a;

  signedArea *= 0.5;
  centroid.point.x /= (6.0 * signedArea);
  centroid.point.y /= (6.0 * signedArea);

  return centroid;
}

geometry_msgs::PointStamped polygonAnalise::getPolygonCentroid()
{
  return centroid;
}
}

namespace occupancy_analise
{
class occupancyAnalise
{
public:
  occupancyAnalise(string topicName, string frame_id);

  void occupancyDataTreatment(const nav_msgs::OccupancyGrid::ConstPtr& ocGrid);

private:
  ros::NodeHandle n;

  ros::Subscriber occupancySub;

  string topicName;
  string frameId;
};

occupancyAnalise::occupancyAnalise(string topicName, string frame_id)
{
  this->topicName = topicName;
  this->frameId = frame_id;

  occupancySub = n.subscribe(topicName, 1000, &occupancyAnalise::occupancyDataTreatment, this);
  ROS_INFO("Topic %s subscribed!", topicName.c_str());
}

void occupancyAnalise::occupancyDataTreatment(const nav_msgs::OccupancyGrid::ConstPtr& ocGrid)
{
}
}

/**
   @brief Main function to compute the algorithms for local path planning
   @param argc
   @param argv
   @return int
 */
int main(int argc, char** argv)
{
  /*-- Node initialization --*/
  ros::init(argc, argv, "local_path_planning");

  ros::NodeHandle n;  // node "local_path_planning" access point

  polygon_analise::polygonAnalise polygonAnalise("merged_polygon", "/map");

  occupancy_analise::occupancyAnalise occupancyAnalise("ocupancy_grid", "/map");

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
