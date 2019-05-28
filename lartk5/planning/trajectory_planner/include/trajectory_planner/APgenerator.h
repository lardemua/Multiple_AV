#ifndef _APgenerator_H_
#define _APgenerator_H_


#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <math.h>
#include <stdio.h>
#include <trajectory_planner/coordinates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <numeric>
#include <vector>
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include <fstream>
#include <stdlib.h> /* atof */
#include <string>

#include <mtt/TargetListPC.h>
// #include <pcl/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// namepaces
using namespace visualization_msgs;

void ExtractVel(gazebo_msgs::ModelStates models);
// void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
// Marker makeBox(InteractiveMarker &msg);
// InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg);
// InteractiveMarker make6DofMarker(bool fixed);
// void makeMarker(trajectory_planner::coordinates message);
// void ExtractPose(gazebo_msgs::ModelStates models);
// void CalcAP(gazebo_msgs::ModelStates models);
void line_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input);
// void ReadFile();
void CreateAPMarker(trajectory_planner::coordinates message);


#ifdef _APgenerator_CPP_
#define _EXTERN_
#else
#define _EXTERN_ extern
#endif

_EXTERN_ ros::NodeHandle *p_n;
// tf::TransformListener *p_listener;
// _EXTERN_ tf::TransformListener *p_listener;
// _EXTERN_ tf::StampedTransform *transform_mtt;
// _EXTERN_ tf::TransformBroadcaster *mtt_broadcaster;

#endif