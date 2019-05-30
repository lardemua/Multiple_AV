// #include <interactive_markers/interactive_marker_server.h>
// #include <interactive_markers/menu_handler.h>
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
// #include <fstream>
// #include <stdlib.h> /* atof */
// #include <string>

// #include <atlasmv_base/AtlasmvMotionCommand.h>
// #include <atlasmv_base/AtlasmvStatus.h>
// #include <geometry_msgs/Polygon.h>
// #include <geometry_msgs/PolygonStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <math.h>
#include <mtt/TargetListPC.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// #include <trajectory_planner/c_manage_trajectory.h>
// #include <trajectory_planner/c_trajectory.h>
// #include <trajectory_planner/coordinates.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// #include <ackermann_msgs/AckermannDrive.h>
// #include <geometry_msgs/Twist.h>

// namepaces
// using namespace visualization_msgs;

// Global Vars
// ros::NodeHandle *p_n;
trajectory_planner::coordinates message_ap;

std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptrl(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> pct2;

tf::TransformBroadcaster mtt_broadcaster;
tf::TransformListener listener;
tf::StampedTransform transform_mtt;

ros::Publisher ap_pub;
ros::Subscriber line_sub;
ros::Subscriber model_states;

double speed_new = 0.000000001;

void ExtractVel(gazebo_msgs::ModelStates models)
{
  double velX = models.twist[1].linear.x;
  double velY = models.twist[1].linear.y;

  speed_new = sqrt(pow(models.twist[1].linear.x, 2) + pow(models.twist[1].linear.y, 2));
  // ROS_INFO("x= %f, y= %f", pose[0], pose[1]);
}

void line_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_v_ptrl);

  pc_v2.erase(pc_v2.begin(), pc_v2.end());

  pc_v2.push_back(*pc_v_ptrl);
  // plan_trajectory = true;
  ros::NodeHandle nh;
  double SPEED_REQUIRED;
  nh.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
  double APdist;
  nh.getParam("Param/APdist", APdist);

  double max_dist_AP = APdist / (SPEED_REQUIRED / speed_new);

  double dist_max = 0.0;
  int point_chosen_AP = 10000;

  double ap_x, ap_y;

  for (size_t i = 0; i < pc_v2.size(); ++i) //std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2 -> contem a linha central;
  {

    ros::NodeHandle nh;
    int car_number = 0;
    nh.getParam("car_number", car_number);
    char car_name[20] = "/vehicle_odometry_";
    char car_number_string[2];
    sprintf(car_number_string, "%d", car_number);
    strcat(car_name, car_number_string);

    if (i == 0)
    {
      try
      {
        //Get the transform between two frames by frame ID.
        //pc_v2[i].header.frame_id -> The frame to which data should be transformed
        //"/vehicle_odometry" -> The frame where the data originated
        // ros::Time(0) -> The time at which the value of the transform is desired. (0 will get the latest)
        //transform_mtt -> The transform reference to fill.
        ros::Time time = ros::Time::now();

        listener.lookupTransform(pc_v2[i].header.frame_id, car_name, ros::Time(0), transform_mtt);

        //Send a StampedTransform The stamped data structure includes frame_id, and time, and parent_id already.
        // mtt_broadcaster.sendTransform(tf::StampedTransform(transform_mtt, time, pc_v2[i].header.frame_id, "/vehicle_odometry"));

        ros::spinOnce();
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    //Apply a rigid transform defined by a 3D offset and a quaternion.
    //pc_v2[i] -> cloud_in
    //pct2 -> cloud_out
    //transform_mtt.inverse() -> a rigid transformation from tf
    pcl_ros::transformPointCloud(pc_v2[i], pct2, transform_mtt.inverse()); //transform_mtt -> tf entre "pc_v2[i].header.frame_id" e "/vehicle_odometry"

    // ROS_INFO("pct2.x: %f, pct2.y: %f", pct2.x, pct2.y);
    for (size_t i = 0; i < pct2.points.size(); ++i)
    {
      double point_dist = sqrt(pow(pct2.points[i].x, 2) + pow(pct2.points[i].y, 2));
      if (point_dist < max_dist_AP && point_dist > dist_max)
      {
        dist_max = point_dist;
        point_chosen_AP = i;

        ap_x = pct2.points[i].x;
        ap_y = pct2.points[i].y;
      }
    }
  }
  ROS_INFO("point_chosen_AP: %d, dist_max: %f", point_chosen_AP, dist_max);

  message_ap.x = ap_x;
  message_ap.y = ap_y;
  message_ap.theta = 0;

  message_ap.header.stamp = ros::Time::now();
  message_ap.header.frame_id = "/world";

  ap_pub.publish(message_ap);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "APgenerator");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  // p_n = &n;

  line_sub = n.subscribe("/line_pcl", 1000, line_callback);

  model_states = n.subscribe("/gazebo/model_states", 1, ExtractVel);

  ap_pub = n.advertise<trajectory_planner::coordinates>("/AP", 1000);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}