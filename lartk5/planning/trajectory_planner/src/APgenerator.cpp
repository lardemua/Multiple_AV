/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
 * @file APgenerator.cpp
 * @brief Interactive marker atractor point generator
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file APgenerator.cpp
 * @brief Interactive marker atractor point generator
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

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

// namepaces
using namespace visualization_msgs;

// Global Vars
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::NodeHandle *p_n;
ros::Publisher coor_pub;
ros::Publisher ap_pub;
ros::Subscriber model_states;
trajectory_planner::coordinates message;
trajectory_planner::coordinates message_ap;
ros::Publisher ap_marker;
std::vector<double> pose{0, 0, 0};
std::vector<double> previousWaypoint{0, 0};
std::vector<double> nextWaypoint{0, 0};
double waypoints[291175][2];
bool firstIter = true;
int currentWaypoint = 0;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // ROS_INFO("Here");
  if (feedback->mouse_point_valid)
  {
    // mouse_point_ss << " at " << feedback->mouse_point.x
    //<< ", " << feedback->mouse_point.y
    //<< ", " << feedback->mouse_point.z
    //<< " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    message.x = feedback->pose.position.x;
    message.y = feedback->pose.position.y;
    message.theta = 2.0 * asin(feedback->pose.orientation.z);

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/world";

    coor_pub.publish(message);

    break;
  }

  server->applyChanges();
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale / 2;
  marker.scale.y = msg.scale / 6;
  marker.scale.z = msg.scale / 6;
  marker.color.r = 0;
  marker.color.g = 0.7;
  marker.color.b = 0;
  marker.color.a = 0.6;
  return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

InteractiveMarker make6DofMarker(bool fixed)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/world";
  int_marker.pose.position.x = 6;
  int_marker.pose.position.y = 0;
  int_marker.pose.position.z = 0;
  int_marker.scale = 0.6;

  int_marker.name = "Control target";
  int_marker.description = "Control the final \nposition of the robot";

  // insert a box
  makeBoxControl(int_marker);
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.name = "rotate_z";
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  //---------------------------------------

  message.x = 6;
  message.y = 0;
  message.theta = 0;

  message.header.stamp = ros::Time::now();
  message.header.frame_id = "/world";

  coor_pub.publish(message);

  //----------------------------------------

  return int_marker;
}

void makeMarker(trajectory_planner::coordinates message)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "AP";
  marker.header.stamp = ros::Time();
  marker.ns = "AP";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = message.x;
  marker.pose.position.y = message.y;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.text = "HERE!";
  ap_marker.publish(marker);
}

void ExtractPose(gazebo_msgs::ModelStates models)
{
  pose[0] = models.pose[1].position.x;
  pose[1] = models.pose[1].position.y;
  pose[2] = models.pose[1].orientation.z;
  // ROS_INFO("x= %f, y= %f", pose[0], pose[1]);
}

void CalcAP(gazebo_msgs::ModelStates models)
{

  ExtractPose(models);

  if (firstIter == true)
  {
    double Nx = 1000.0;
    double Px = 1000.0;
    firstIter = false;

    if (abs(pose[2]) < M_PI_4) // 1: direção: x->
    {
      for (int w = 0; w < (sizeof(waypoints)[0]); w++)
      {
        double newDiff = pose[0] - waypoints[w][0];
        if (newDiff < 0 && abs(newDiff) < Px)
        {
          Px = abs(newDiff);

          nextWaypoint[0] = waypoints[w][0];
          nextWaypoint[1] = waypoints[w][1];

          currentWaypoint = w;

          previousWaypoint[0] = waypoints[w - 1][0];
          previousWaypoint[1] = waypoints[w - 1][1];
        }
      }
    }
    // else if (abs(pose[2]) > 3 * M_PI_4) // 2: direção: <-x
    // {
    // }
    // else if (pose[2] >= M_PI_4 && pose[2] <= 3 * M_PI_4) // 3: direção: y->
    // {
    // }
    // else if (pose[2] >= 5 * M_PI_4 && pose[2] <= 7 * M_PI_4) // 4: direção: <-y
    // {
    // }
    else
    {
      ROS_INFO("ERROR with the orientation of the model!");
    }
  }

  

  double distFront = sqrt(pow((pose[0] - nextWaypoint[0]), 2) + pow((pose[1] - nextWaypoint[1]), 2));
  double distBack = sqrt(pow((pose[0] - previousWaypoint[0]), 2) + pow((pose[1] - previousWaypoint[1]), 2));

  if (distFront < distBack)
  {
    currentWaypoint++;
    nextWaypoint[0] = waypoints[currentWaypoint][0];
    nextWaypoint[1] = waypoints[currentWaypoint][1];
    ROS_INFO("New waypoint= %f,%f", nextWaypoint[0], nextWaypoint[1]);
  }

  double distX= nextWaypoint[0] - pose[0];
  double distY= nextWaypoint[1] - pose[1];

  ROS_INFO("distFront= %f, distBack= %f", distFront, distBack);

  message.x = distX;
  message.y = distY;
  message.theta = 0;

  message.header.stamp = ros::Time::now();
  message.header.frame_id = "/world";

  ap_pub.publish(message);

  makeMarker(message);
}

void ReadFile()
{
  std::ifstream sourcefile("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/XeY1000.csv");

  if (!sourcefile.is_open())
    std::cout << "ERROR: File Open" << '\n';

  std::string posX;
  std::string posY;
  double pos_x, pos_y;
  int count = 0;

  while (sourcefile.good())
  {

    getline(sourcefile, posX, ',');
    getline(sourcefile, posY, '\n');

    pos_x = atof(posX.c_str());
    pos_y = atof(posY.c_str());

    waypoints[count][0] = pos_x;
    waypoints[count][1] = pos_y;

    // ROS_INFO("READ FILE");

    count++;
  }
  // ROS_INFO("READ FILE");

  sourcefile.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "APgenerator");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  p_n = &n;

  ReadFile();

  coor_pub = n.advertise<trajectory_planner::coordinates>("/msg_coordinates", 1000);

  ap_pub = n.advertise<trajectory_planner::coordinates>("/AP", 1000);

  model_states = n.subscribe("/gazebo/model_states", 1, CalcAP);

  ap_marker = n.advertise<visualization_msgs::Marker>("/ap_marker", 0);

  server.reset(new interactive_markers::InteractiveMarkerServer("APgenerator/im", "", false));
  ros::Duration(0.1).sleep();
  InteractiveMarker marker = make6DofMarker(true);

  server->applyChanges();

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  server.reset();
}
