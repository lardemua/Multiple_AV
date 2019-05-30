// #ifndef _APgenerator_CPP_
#define _APgenerator_CPP_

#include <trajectory_planner/APgenerator.h>

using namespace visualization_msgs;

tf::TransformListener *p_listener;
tf::StampedTransform *transform_mtt;
tf::TransformBroadcaster *mtt_broadcaster;

// Global Vars
ros::Publisher apoint_pub;
ros::Subscriber line_sub;
ros::Subscriber model_states_2;
ros::Publisher ap_marker;
trajectory_planner::coordinates message;
trajectory_planner::coordinates message_ap;
bool firstIter = true;
int currentWaypoint = 0;
double speed_min = 0.000000001;
double this_speed_new = 0;
double this_pos_x = 0;
double this_pos_y = 0;
int count_ap_y = 0;
double ap_y_temp = (2.650925) / 10;

std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptrl(new pcl::PointCloud<pcl::PointXYZ>);

void ExtractVel(gazebo_msgs::ModelStates models)
{
  int pos_name = 0;
  ros::NodeHandle n;
  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "cirkit_unit03_h";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  double velX;
  double velY;

  double pos_other[models.name.size()-2][2];
  double vel_other[models.name.size()-2];

  for (int n = 1; n < models.name.size(); n++) //n=0 => track
  {
    if (models.name[n] == car_name)
    {
      // ROS_INFO("car_name: %s", car_name);
      // velX = models.twist[n].linear.x;
      // velY = models.twist[n].linear.y;
      // ROS_INFO("velX: %f, velY: %f", velX, velY);
      this_pos_x = models.pose[n].position.x;
      this_pos_y = models.pose[n].position.y;
      this_speed_new = sqrt(pow(models.twist[n].linear.x, 2) + pow(models.twist[n].linear.y, 2));
    }
    else
    {
      // velX = models.twist[n].linear.x;
      // velY = models.twist[n].linear.y;
      pos_other[n][0] = models.pose[n].position.x;
      pos_other[n][1] = models.pose[n].position.y;
      // double dist = sqrt(pow(models.pose[n].position.x, 2) + pow(models.pose[n].position.y, 2));
      vel_other[n] = sqrt(pow(models.twist[n].linear.x, 2) + pow(models.twist[n].linear.y, 2));
    }
  }

  

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
  double APdistMax; // 10
  nh.getParam("Param/APdistMax", APdistMax);
  double APdistMin; // 5
  nh.getParam("Param/APdistMin", APdistMin);

  int car_number = 0;
  nh.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  double max_dist_AP;

  if (this_speed_new != 0)
  {
    // max_dist_AP = APdistMax / (SPEED_REQUIRED / this_speed_new);
    max_dist_AP = (APdistMax - APdistMin) * (this_speed_new / SPEED_REQUIRED);
    max_dist_AP = APdistMin + max_dist_AP;
  }
  else
  {
    max_dist_AP = APdistMax;
  }

  if (max_dist_AP < APdistMin * 1.2)
  {
    max_dist_AP = APdistMin * 1.2;
  }

  double dist_max_reached = APdistMin;
  int point_chosen_AP;

  double ap_x = 6;
  double ap_y = 0;

  int count_ap = 0;

  // ROS_INFO("max_dist_AP: %f, dist_max_reached: %f", max_dist_AP, dist_max_reached); // 1,679380    0.200000

  for (size_t i = 0; i < pc_v2.size(); ++i) //std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2 -> contem a linha central;
  {
    if (i == 0)
    {
      try
      {
        //Get the transform between two frames by frame ID.
        //pc_v2[i].header.frame_id -> The frame to which data should be transformed
        //"/vehicle_odometry" -> The frame where the data originated
        // ros::Time(0) -> The time at which the value of the transform is desired. (0 will get the latest)
        //transform_mtt -> The transform reference to fill.
        ros::Time time;
        time.fromNSec(pc_v2[i].header.stamp);

        // p_listener->lookupTransform(pc_v2[i].header.frame_id, "/vehicle_odometry", time, *transform_mtt);
        p_listener->lookupTransform(pc_v2[i].header.frame_id, car_name, ros::Time(0), *transform_mtt);

        //Send a StampedTransform The stamped data structure includes frame_id, and time, and parent_id already.
        // mtt_broadcaster->sendTransform(tf::StampedTransform(*transform_mtt, time, pc_v2[i].header.frame_id, "/vehicle_odometry"));

        ros::spinOnce();
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }

    pcl::PointCloud<pcl::PointXYZ> pct2;
    //Apply a rigid transform defined by a 3D offset and a quaternion.
    //pc_v2[i] -> cloud_in
    //pct2 -> cloud_out
    //transform_mtt.inverse() -> a rigid transformation from tf
    pcl_ros::transformPointCloud(pc_v2[i], pct2, transform_mtt->inverse()); //transform_mtt -> tf entre "pc_v2[i].header.frame_id" e "/vehicle_odometry"

    for (size_t i = 0; i < pct2.points.size(); ++i)
    {

      if (pct2.points[i].x > 0)
      {
        // ROS_INFO("pct2.x: %f, pct2.y: %f", pct2.points[i].x, pct2.points[i].y);
        double point_dist = sqrt(pow(pct2.points[i].x, 2) + pow(pct2.points[i].y, 2));
        if (point_dist < max_dist_AP && point_dist > dist_max_reached)
        {
          dist_max_reached = point_dist;
          point_chosen_AP = i;
          count_ap++;

          ap_x = pct2.points[i].x;
          ap_y = pct2.points[i].y;
        }
      }
    }
  }
  // ROS_INFO("count_ap: %d", count_ap);                                                       //30
  // ROS_INFO("point_chosen_AP: %d, dist_max_reached: %f", point_chosen_AP, dist_max_reached); // 100    1.654483
  // ROS_INFO("ap_x: %f, ap_y: %f", ap_x, ap_y);                                               //
  // ROS_INFO("");

  //------------------------------------------------------------------------------------------------------
  //---------------------Create Dynamic AP----------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------

  bool _LINES_;
  bool _OVERTAKING_;
  nh.getParam("Param/LINES", _LINES_);
  nh.getParam("Param/OVERTAKING", _OVERTAKING_);

  if (_OVERTAKING_ == true)
  {
    ap_y = ap_y + 2.650925;
    nh.setParam("Param/LINES", false);
    // if (count_ap_y < 10)
    // {
    //   ap_y = ap_y + (ap_y_temp * count_ap_y);
    //   count_ap_y++;
    // }
  }
  else if (_LINES_ == true)
  {
    ap_y = ap_y - 2.650925;
  }

  //------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------

  message_ap.x = ap_x;
  message_ap.y = ap_y;
  message_ap.theta = 0;

  // !!!! Change this to some time
  message_ap.header.stamp = ros::Time(0);
  message_ap.header.frame_id = "/world";

  apoint_pub.publish(message_ap);

  CreateAPMarker(message_ap);
}

void CreateAPMarker(trajectory_planner::coordinates message)
{
  ros::NodeHandle nh;
  int car_number = 0;
  nh.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  visualization_msgs::Marker marker;
  marker.header.frame_id = car_name;
  marker.header.stamp = ros::Time(0);
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
  marker.scale.y = 1;
  marker.scale.z = 0.01;
  marker.color.a = 1; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  ap_marker.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "APgenerator");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  p_n = &n;

  mtt_broadcaster = new (tf::TransformBroadcaster);
  transform_mtt = new (tf::StampedTransform);
  p_listener = new (tf::TransformListener);

  line_sub = n.subscribe("/line_pcl", 1000, line_callback);

  apoint_pub = n.advertise<trajectory_planner::coordinates>("/Apoint", 1000);

  model_states_2 = n.subscribe("/gazebo/model_states", 1, ExtractVel);

  ap_marker = n.advertise<visualization_msgs::Marker>("/ap_marker", 0);

  ros::Duration(0.1).sleep();

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}