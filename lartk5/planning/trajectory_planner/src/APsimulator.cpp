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
trajectory_planner::coordinates message;
std::vector<double> pose{0, 0};
std::vector<double> previousWaypoint{0, 0};
std::vector<double> nextWaypoint{0, 0};
double waypoints[291175][2];

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

void ExtractPose(gazebo_msgs::ModelStates models)
{
  pose[0] = models.pose[1].position.x;
  pose[1] = models.pose[1].position.y;

  // ROS_INFO("x= %f, y= %f", pose[0], pose[1]);
}

void CalcAP(){

  

}

void ReadFile()
{
  std::ifstream sourcefile("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/XeY.csv");

  if(!sourcefile.is_open()) std::cout << "ERROR: File Open" << '\n';

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

  coor_pub = n.advertise<trajectory_planner::coordinates>("/AP", 1000);

  ros::Subscriber model_states = n.subscribe("/gazebo/model_states", 1, ExtractPose);

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