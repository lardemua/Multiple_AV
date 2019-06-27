/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro -
http://lars.mec.ua.pt All rights reserved.
 Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
        *Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer. *Redistributions in binary
form must reproduce the above copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other materials provided with
the distribution. *Neither the name of the University of Aveiro nor the names of
its contributors may be used to endorse or promote products derived from this
software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
#ifndef _trajectory_planner_nodelet_CPP_
#define _trajectory_planner_nodelet_CPP_

/**
 * @file trajectory_planner_nodelet.cpp
 * @brief Uses the c-trajectory class, to publish trajectories and send the
 * message to follow one of them
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file trajectory_planner_nodelet.cpp
 * @brief Uses the c-trajectory class, to publish trajectories and send the
 * message to follow one of them
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

#include <trajectory_planner/trajectory_planner_nodelet.h>

bool plan_trajectory = false;
bool have_plan = false;
bool have_trajectory = false;
vector<double> last_dir{0, 0, 0, 0, 0};
geometry_msgs::PoseStamped pose_in;
geometry_msgs::PoseStamped pose_transformed;

double max_angle = 41.5;

std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v;
//-------------------------------------------------------------------------------------------//
std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v1;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptr(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_v_ptrl(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher ap_marker_coll;
ros::Publisher ap_marker_coll_back;
ros::Publisher ap_marker_coll_space;
ros::Publisher ap_marker_coll_space_back;

ros::Publisher mindist_pub;
ros::Publisher traj_pub;
ros::Publisher ap_pub;
ros::Publisher DS_pub;

double this_speed_new = 0;
double this_pos_x = 0;
double this_pos_y = 0;

/**
 * @brief extracts speed from model in Gazebo
 * 
 * @param models -> model in analysis
 */
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

  for (int n = 1; n < models.name.size(); n++) //n=0 => track
  {
    if (models.name[n] == car_name)
    {
      this_pos_x = models.pose[n].position.x;
      this_pos_y = models.pose[n].position.y;
      this_speed_new = sqrt(pow(models.twist[n].linear.x, 2) + pow(models.twist[n].linear.y, 2));
    }
  }

  // ROS_INFO("x= %f, y= %f", pose[0], pose[1]);
}

/**
 * @brief Set attractor point coordinates
 * @param trajectory_planner::coordinates msg
 * @return void
 */
void set_coordinates(trajectory_planner::coordinates msg)
{
  // 	cout<<"stat Message received!!!"<<endl;
  // Change parameters
  // pose_in.pose.position.x = msg.x;
  // pose_in.pose.position.y = msg.y;
  // pose_in.pose.orientation.w = cos(msg.theta / 2.0);
  // pose_in.pose.orientation.x = 0.0;
  // pose_in.pose.orientation.y = 0.0;
  // pose_in.pose.orientation.z = sin(msg.theta / 2.0);

  pose_in.pose.position.x = msg.x;
  pose_in.pose.position.y = msg.y;
  pose_in.pose.orientation.w = cos(msg.theta / 2.0);
  pose_in.pose.orientation.x = 0.0;
  pose_in.pose.orientation.y = 0.0;
  pose_in.pose.orientation.z = sin(msg.theta / 2.0);

  pose_in.header.frame_id = "/world";

  manage_vt->set_attractor_point(msg.x, msg.y, msg.theta);
  plan_trajectory = true;
}

/**
 * @brief Set mtt points
 * @param mtt::TargetListPC msg
 * @return void
 */
void mtt_callback(mtt::TargetListPC msg)
{
  // ROS_INFO("received mtt num obs=%ld num lines first obs =%d frame_id=%s",
  // msg.id.size(), msg.obstacle_lines[0].width,
  //  msg.header.frame_id.c_str());

  // copy to global variable
  pc_v.erase(pc_v.begin(), pc_v.end());
  for (size_t i = 0; i < msg.id.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(msg.obstacle_lines[i], tmp);
    tmp.header.frame_id = msg.header.frame_id;
    // 		tmp.header.stamp=msg.header.stamp;

    // ROS_INFO("Obstacle %ld has %ld lines", i, tmp.points.size());
    pc_v.push_back(tmp);
  }
}

/**
 * @brief Set polygon points
 * @param mtt::TargetListPC msg
 * @return void
 */
void pcl_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
  //Convertions
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_v_ptr);

  //Erase PointCloud pc_v1
  pc_v1.erase(pc_v1.begin(), pc_v1.end());

  //Insert a new point in the cloud, at the end of the container.(This breaks the organized structure of the cloud by setting the height to 1!)
  pc_v1.push_back(*pc_v_ptr);

  // plan_trajectory = true;
}

/**
 * @brief Set line points
 * @param mtt::TargetListPC msg
 * @return void
 */
void line_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_v_ptrl);

  pc_v2.erase(pc_v2.begin(), pc_v2.end());

  pc_v2.push_back(*pc_v_ptrl);
  // plan_trajectory = true;
}

/**
 * @brief Function that generates a few number of trajectories from the defines
 * _NUM_TRAJ_, _NUM_NODES_ and _TRAJECTORY_ANGLE_
 * @param vehicle speed
 * @return void
 */
void velocity_callback(double speed) //speed = SPEED_SAFFETY
{

  ros::NodeHandle n;
  double MAX_STEERING_ANGLE;
  n.getParam("Param/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  double MIN_STEERING_ANGLE;
  n.getParam("Param/MIN_STEERING_ANGLE", MIN_STEERING_ANGLE);
  // double TRAJECTORY_ANGLE;
  // n.getParam("Param/TRAJECTORY_ANGLE", TRAJECTORY_ANGLE); //3
  int NUM_TRAJ;
  n.getParam("Param/NUM_TRAJ", NUM_TRAJ);
  double NUM_NODES;
  n.getParam("Param/NUM_NODES", NUM_NODES);
  double SPEED_REQUIRED;
  n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
  double TRAJ_DENSITY;
  n.getParam("Param/TRAJ_DENSITY", TRAJ_DENSITY);

  double max_dist = pow(speed * 3.6, 2) / 100; //distancia de travagem?
  if (max_dist > 20)
  {
    max_dist = 20;
  }
  if (max_dist < 4.5)
  {
    max_dist = 4.5;
  }

  if (this_speed_new > speed) //speed=SPEED_SAFFETY
  {
    max_angle = MAX_STEERING_ANGLE - (MAX_STEERING_ANGLE - MIN_STEERING_ANGLE) * (this_speed_new / SPEED_REQUIRED);
  }

  double iter_value = max_angle / (pow(TRAJ_DENSITY, NUM_TRAJ));
  int n_traj = 1;
  double i = 0.00000001;
  while (i < max_angle)
  {
    if (i == 0.00000001)
    {
      vector<double> v_a;
      vector<double> v_arc;
      for (int j = 0; j < NUM_NODES; ++j) //criar tantos quanto NUM_NODES
      {
        v_a.push_back(M_PI / 180. * i);        //sempre igual
        v_arc.push_back(max_dist / NUM_NODES); //sempre igual
      }
      manage_vt->create_new_trajectory(v_a, v_arc, v_a);
      i = iter_value;
    }
    else
    {
      vector<double> v_a1;
      vector<double> v_arc1;
      for (int j = 0; j < NUM_NODES; ++j) //criar tantos quanto NUM_NODES
      {
        v_a1.push_back(M_PI / 180. * i);        //sempre igual
        v_arc1.push_back(max_dist / NUM_NODES); //sempre igual
      }
      manage_vt->create_new_trajectory(v_a1, v_arc1, v_a1);

      vector<double> v_a2;
      vector<double> v_arc2;
      for (int j = 0; j < NUM_NODES; ++j)
      {
        v_a2.push_back(M_PI / 180. * (-i));
        v_arc2.push_back(max_dist / NUM_NODES);
      }
      manage_vt->create_new_trajectory(v_a2, v_arc2, v_a2);
      i = i * TRAJ_DENSITY;
    }

    n_traj++;
  }

  have_trajectory = true;
}

/**
 * @brief Function that updates trajectories during the planning
 * @param vehicle speed
 * @return void
 */
void velocity_update_callback(double speed) //speed=SPEED_SAFFETY
{

  ros::NodeHandle n;
  double MAX_STEERING_ANGLE;
  n.getParam("Param/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  double MIN_STEERING_ANGLE;
  n.getParam("Param/MIN_STEERING_ANGLE", MIN_STEERING_ANGLE);
  // double TRAJECTORY_ANGLE;
  // n.getParam("Param/TRAJECTORY_ANGLE", TRAJECTORY_ANGLE);
  double NUM_NODES;
  n.getParam("Param/NUM_NODES", NUM_NODES);
  int NUM_TRAJ;
  n.getParam("Param/NUM_TRAJ", NUM_TRAJ);
  double SPEED_REQUIRED;
  n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
  double TRAJ_DENSITY;
  n.getParam("Param/TRAJ_DENSITY", TRAJ_DENSITY);

  double max_dist = pow(speed * 3.6, 2) / 100; //3.6? -> tamanho do veiculo?
  if (max_dist > 20)
  {
    max_dist = 20;
  }
  if (max_dist < 4.5)
  {
    max_dist = 4.5;
  }

  if (this_speed_new > speed) //speed=SPEED_SAFFETY
  {
    max_angle = MAX_STEERING_ANGLE - (MAX_STEERING_ANGLE - MIN_STEERING_ANGLE) * (this_speed_new / SPEED_REQUIRED);
  }

  double iter_value = max_angle / (pow(TRAJ_DENSITY, NUM_TRAJ));
  double i = 0.00000001;
  int num_trajec = 0;

  while (i < max_angle)
  {
    if (i == 0.00000001)
    {
      vector<double> v_a;
      vector<double> v_arc;
      for (int j = 0; j < NUM_NODES; ++j)
      {
        v_a.push_back(M_PI / 180. * i); //envio do angulo em radianos
        v_arc.push_back(max_dist / NUM_NODES);
      }
      manage_vt->update_trajectory(v_a, v_arc, v_a, num_trajec);
      num_trajec = num_trajec + 1;
      i = iter_value;
    }
    else
    {
      vector<double> v_a1;
      vector<double> v_arc1;
      for (int j = 0; j < NUM_NODES; ++j)
      {
        v_a1.push_back(M_PI / 180. * i);
        v_arc1.push_back(max_dist / NUM_NODES);
      }
      manage_vt->update_trajectory(v_a1, v_arc1, v_a1, num_trajec);
      num_trajec = num_trajec + 1;

      //angulo na outra direcao
      vector<double> v_a2;
      vector<double> v_arc2;
      for (int j = 0; j < NUM_NODES; ++j)
      {
        v_a2.push_back(M_PI / 180. * (-i));
        v_arc2.push_back(max_dist / NUM_NODES);
      }
      manage_vt->update_trajectory(v_a2, v_arc2, v_a2, num_trajec);
      num_trajec = num_trajec + 1;
      i = i * TRAJ_DENSITY;
    }
    // i = i + TRAJECTORY_ANGLE;
  }

  have_trajectory = true;
}

/**
 * @brief Sets the speed in function of chosen trajectory angle
 * @param chosen trajectory angle
 * @return speed
 */
double angle_to_speed(double angle)
{

  ros::NodeHandle n;
  double SPEED_REQUIRED;
  double SPEED_SAFFETY;
  n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
  n.getParam("Param/SPEED_SAFFETY", SPEED_SAFFETY);
  // double MAX_STEERING_ANGLE;
  // n.getParam("Param/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);

  double speed_out = SPEED_REQUIRED - ((SPEED_REQUIRED - SPEED_SAFFETY) / (max_angle * M_PI / 180)) * abs(angle);
  return speed_out;
  // plan_trajectory = true;
}

/**
 * @brief Last five chosen directions filter
 * @param last chosen direction
 * @return mean
 */
double compute_last_dir(double angle)
{
  int n = last_dir.size();
  vector<double> last_dir_cp = last_dir;
  last_dir.clear();

  for (size_t i = 1; i < n; i++)
  {
    last_dir.push_back(last_dir_cp.at(i));
  }
  last_dir.push_back(angle);

  double mean = accumulate(last_dir.begin(), last_dir.end(), 0.0) / n;
  return mean;
}

/**
 * @brief Publisher of the detection space (front)
 * 
 * @param limit_left -> max (y) pos for the space
 * @param limit_right -> min (y) pos for the space
 * @param DetectDist -> max dist of detection from Param
 */
void PublishCollSpace(double limit_left, double limit_right, double DetectDist)
{

  ros::NodeHandle n;
  p_n = &n;
  // n.getParam("Param/simul", _simulation_);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = car_name;
  line_strip.ns = "colision_space";
  line_strip.header.stamp = ros::Time::now();
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = 0;
  p1.y = limit_left;
  p1.z = 0;

  geometry_msgs::Point p2;
  p2.x = 0;
  p2.y = limit_right;
  p2.z = 0;

  geometry_msgs::Point p3;
  p3.x = DetectDist;
  p3.y = limit_right;
  p3.z = 0;

  geometry_msgs::Point p4;
  p4.x = DetectDist;
  p4.y = limit_left;
  p4.z = 0;

  geometry_msgs::Point p5;
  p5.x = 0;
  p5.y = limit_left;
  p5.z = 0;

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  line_strip.points.push_back(p3);
  line_strip.points.push_back(p4);
  line_strip.points.push_back(p5);

  ap_marker_coll_space.publish(line_strip);
}

/**
 * @brief Publisher of the detection space (back)
 * 
 * @param limit_left -> max (y) pos for the space
 * @param limit_right -> min (y) pos for the space
 * @param DetectDist -> max dist of detection from Param
 */
void PublishCollSpace_BACK(double limit_left, double limit_right, double DetectDist)
{

  ros::NodeHandle n;
  p_n = &n;
  // n.getParam("Param/simul", _simulation_);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = car_name;
  line_strip.ns = "colision_space";
  line_strip.header.stamp = ros::Time::now();
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.r = 0.5;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.5;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = 0;
  p1.y = limit_left;
  p1.z = 0;

  geometry_msgs::Point p2;
  p2.x = 0;
  p2.y = limit_right;
  p2.z = 0;

  geometry_msgs::Point p3;
  p3.x = -DetectDist / 2;
  p3.y = limit_right;
  p3.z = 0;

  geometry_msgs::Point p4;
  p4.x = -DetectDist / 2;
  p4.y = limit_left;
  p4.z = 0;

  geometry_msgs::Point p5;
  p5.x = 0;
  p5.y = limit_left;
  p5.z = 0;

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  line_strip.points.push_back(p3);
  line_strip.points.push_back(p4);
  line_strip.points.push_back(p5);

  ap_marker_coll_space_back.publish(line_strip);
}

/**
 * @brief Publisher of the detected points (front)
 * 
 * @param points_detected_2 -> PointCloud with the detected points
 */
void PublishColl(pcl::PointCloud<pcl::PointXYZRGBA> points_detected_2)
{

  ros::NodeHandle n;
  p_n = &n;
  // n.getParam("Param/simul", _simulation_);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);
  points_detected_2.header.frame_id = car_name;

  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(points_detected_2, pc2);

  ap_marker_coll.publish(pc2);
}

/**
 * @brief Publisher of the detected points (back)
 * 
 * @param points_detected_3 -> PointCloud with the detected points
 */
void PublishColl_BACK(pcl::PointCloud<pcl::PointXYZRGBA> points_detected_3)
{

  ros::NodeHandle n;
  p_n = &n;
  // n.getParam("Param/simul", _simulation_);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);
  points_detected_3.header.frame_id = car_name;

  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(points_detected_3, pc2);

  ap_marker_coll_back.publish(pc2);
}

void Publish_analysis_data(c_manage_trajectoryPtr &manage_vt)
{

  // ros::Time begin = ros::Time::now();
  // std::cout << "begin" << begin << std::endl;

  std_msgs::Float64MultiArray mindist_msg;
  mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mindist_msg.layout.dim[0].label = "sim";
  mindist_msg.layout.dim[0].size = 2;
  mindist_msg.layout.dim[0].stride = 2;
  mindist_msg.layout.data_offset = 0;

  if (manage_vt->chosen_traj.index != -1)
  {
    double alpha_degrees = manage_vt->chosen_traj.alpha * (180 / M_PI);
    // vector<int> vec1 = {manage_vt->chosen_traj.index};
    vector<double> vec1 = {manage_vt->chosen_traj.min_dist,manage_vt->chosen_traj.index, alpha_degrees, max_angle, this_speed_new};
    // vector<double> vec1 = {max_angle};
    mindist_msg.data.clear();
    mindist_msg.data.insert(mindist_msg.data.end(), vec1.begin(), vec1.end());
    // mindist_msg.data.insert(chosen_traj.alpha);
    mindist_pub.publish(mindist_msg);
  }

  // ros::Time end = ros::Time::now();

  // std::cout << "end" << end << std::endl;
}

void Publish_ap_data(c_manage_trajectoryPtr &manage_vt)
{

  std_msgs::Float64MultiArray mindist_msg;
  mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mindist_msg.layout.dim[0].label = "sim";
  mindist_msg.layout.dim[0].size = 2;
  mindist_msg.layout.dim[0].stride = 2;
  mindist_msg.layout.data_offset = 0;

  if (manage_vt->chosen_traj.index != -1)
  {
    vector<double> vec2 = {this_speed_new};
    mindist_msg.data.clear();
    mindist_msg.data.insert(mindist_msg.data.end(), vec2.begin(), vec2.end());
    // mindist_msg.data.insert(chosen_traj.alpha);
    ap_pub.publish(mindist_msg);
  }

  // ros::Time end = ros::Time::now();

  // std::cout << "end" << end << std::endl;
}

void Publish_DS_data(int count_points_detected_front, int count_points_detected_back, double limit_left, double limit_right, double limit_left_back, double limit_right_back, int detect_front, int detect_back, double min_dist_front, double min_dist_back)
{

  std_msgs::Float64MultiArray mindist_msg;
  mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mindist_msg.layout.dim[0].label = "sim";
  mindist_msg.layout.dim[0].size = 2;
  mindist_msg.layout.dim[0].stride = 2;
  mindist_msg.layout.data_offset = 0;

  vector<double> vec2 = {count_points_detected_front, count_points_detected_back, limit_left, limit_right, limit_left_back, limit_right_back, detect_front, detect_back, min_dist_front, min_dist_back};
  mindist_msg.data.clear();
  mindist_msg.data.insert(mindist_msg.data.end(), vec2.begin(), vec2.end());
  // mindist_msg.data.insert(chosen_traj.alpha);
  DS_pub.publish(mindist_msg);

  // ros::Time end = ros::Time::now();

  // std::cout << "end" << end << std::endl;
}

void Publish_traj_data(c_manage_trajectoryPtr &manage_vt)
{

  // ros::Time begin = ros::Time::now();
  // std::cout << "begin" << begin << std::endl;

  std_msgs::Float64MultiArray mindist_msg;
  mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mindist_msg.layout.dim[0].label = "sim";
  mindist_msg.layout.dim[0].size = 2;
  mindist_msg.layout.dim[0].stride = 2;
  mindist_msg.layout.data_offset = 0;

  if (manage_vt->chosen_traj.index != -1)
  {
    double alpha_degrees = manage_vt->chosen_traj.alpha * (180 / M_PI);
    // vector<int> vec1 = {manage_vt->chosen_traj.index};
    vector<double> vec2 = {manage_vt->chosen_traj.index, alpha_degrees, max_angle, this_speed_new};
    mindist_msg.data.clear();
    mindist_msg.data.insert(mindist_msg.data.end(), vec2.begin(), vec2.end());
    // mindist_msg.data.insert(chosen_traj.alpha);
    traj_pub.publish(mindist_msg);
  }

  // ros::Time end = ros::Time::now();

  // std::cout << "end" << end << std::endl;
}

/**
 * @brief Main code of the nodelet
 * @details Publishes the trajectories message and the command message
 * @param int argc
 * @param char **argv
 * @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_planner_nodelet");
  ros::NodeHandle n;
  p_n = &n;
  // n.getParam("Param/simul", _simulation_);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  // Define the publishers and subscribers
  tf::TransformBroadcaster mw_broadcaster;
  tf::TransformBroadcaster mtt_broadcaster;
  tf::TransformListener listener;
  p_listener = &listener;

  ap_marker_coll = n.advertise<sensor_msgs::PointCloud2>("/coll_marker", 100);

  ap_marker_coll_back = n.advertise<sensor_msgs::PointCloud2>("/coll_marker_back", 100);

  ap_marker_coll_space = n.advertise<visualization_msgs::Marker>("/coll_space_marker", 100);

  ap_marker_coll_space_back = n.advertise<visualization_msgs::Marker>("/coll_space_marker_back", 100);

  ros::Publisher array_pub = n.advertise<visualization_msgs::MarkerArray>("/array_of_markers", 1);

  // bool waypoints_bool;
  // n.getParam("Param/WAYPOINTS", waypoints_bool);
  // if (waypoints_bool == true)
  // {
  ros::Subscriber sub = n.subscribe("/Apoint", 1, set_coordinates);
  // }
  // else
  // {
  //   ros::Subscriber sub = n.subscribe("/msg_coordinates", 1, set_coordinates);
  // }

  ros::Subscriber model_states_2 = n.subscribe("/gazebo/model_states", 1, ExtractVel);

  ros::Subscriber clp_sub = n.subscribe("/line_close_point", 1, ExtractCLP2);

  ros::Subscriber mtt_sub = n.subscribe("/mtt_targets", 1, mtt_callback);
  //----------------------------------------------------------------------------------------------------//
  // Polygon stuff
  ros::Subscriber pcl_sub = n.subscribe("/reduced_pcl", 1000, pcl_callback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/steer_drive_controller/cmd_vel", 1);
  mindist_pub = n.advertise<std_msgs::Float64MultiArray>("/analysis_data", 100);

  traj_pub = n.advertise<std_msgs::Float64MultiArray>("/traj_data", 100);

  ap_pub = n.advertise<std_msgs::Float64MultiArray>("/ap_data", 100);

  DS_pub = n.advertise<std_msgs::Float64MultiArray>("/DS_data", 100);

  // road lines
  ros::Subscriber line_sub;
  // if (_simulation_)
  // {
  line_sub = n.subscribe("/line_pcl", 1000, line_callback);
  // }

  int RefreshRate = 30;
  n.getParam("RefreshRate", RefreshRate);

  ros::Rate loop_rate(RefreshRate);

  //   ___________________________________
  //   |                                 |
  //   |        Define the trajectories  |
  //   |_________________________________|
  // Declare the traj manager class
  manage_vt = (c_manage_trajectoryPtr) new c_manage_trajectory();

  // initialize attractor point
  t_desired_coordinates AP;
  AP.x = -1.0;
  AP.y = 0.0;
  AP.theta = -M_PI / 8;
  manage_vt->set_attractor_point(AP.x, AP.y, AP.theta);

  // initialize vehicle description
  manage_vt->set_vehicle_description(_VEHICLE_WIDTH_, _VEHICLE_LENGHT_BACK_, _VEHICLE_LENGHT_FRONT_,
                                     _VEHICLE_HEIGHT_TOP_, _VEHICLE_HEIGHT_BOTTOM_);

  // initialize inter axis distance
  manage_vt->set_inter_axis_distance(_D_);

  double SPEED_SAFFETY;
  n.getParam("Param/SPEED_SAFFETY", SPEED_SAFFETY);

  // initialize trajectories
  velocity_callback(SPEED_SAFFETY);

  //  trajectory information
  // commandPublisher = n.advertise<trajectory_planner::traj_info>("/trajectory_information", 1000);

  while (ros::ok())
  {
    // auto before_traj = std::chrono::high_resolution_clock::now();

    n.getParam("Param/SPEED_SAFFETY", SPEED_SAFFETY);

    double speed = SPEED_SAFFETY;
    // cout << "ros::ok" << endl;
    if (plan_trajectory == true)
    {
      // cout << "plan_trajectory=true" << endl;
      // aqui recebi um comando para defenir uma trajectoria
      have_trajectory = false;

      // benchmark_fn("velocity_update_callback", [&]() { velocity_update_callback(speed); });

      velocity_update_callback(speed); //takes 3ms max

      //-------------------------------------------------------------------------//
      // plan_trajectory = false;
      // ROS_INFO("Going to plan a trajectory");
      // _________________________________
      //|                                 |
      //|    Set the parking view frame   |
      //|_________________________________|
      bool have_transform = true;
      // Set the frame where to draw the trajectories

      // ros::Time time = ros::Time::now();
      ros::Time time = ros::Time(0);
      // while(!p_listener->canTransform("/world", "/vehicle_odometry", time));
      // auto before_try = std::chrono::high_resolution_clock::now();

      try
      {
        // p_listener->lookupTransform("/world", "/vehicle_odometry", time, transformw);

        p_listener->lookupTransform("/world", car_name, ros::Time(0), transformw);
      }
      catch (tf::TransformException ex)
      {
        ROS_INFO("ERRO NO traj2");

        ROS_ERROR("%s", ex.what());
        have_transform = false;
      }

      // auto after_try = std::chrono::high_resolution_clock::now();

      // std::chrono::milliseconds duration_try = std::chrono::duration_cast<std::chrono::milliseconds>(after_try - before_try);

      // std::cout << " try cicle took " << duration_try.count() << "ms." << std::endl;

      if (have_transform & have_trajectory)
      {
        ros::Time time = ros::Time(0);
        ros::spinOnce(); //will call all the callbacks waiting to be called at that point in time.

        // ros::Duration(0.1).sleep();

        //   ___________________________________
        //   |                                 |
        //   |        Trajectory evaluation    |
        //   |_________________________________|

        // Transform attractor point to /vehicle_odometry
        tf::Transformer tt;
        pose_in.header.stamp = time;

        p_listener->transformPose(car_name, pose_in, pose_transformed);

        // Set transformed attractor point
        // manage_vt->set_attractor_point(
        //     pose_transformed.pose.position.x, pose_transformed.pose.position.y,
        //     atan2(2.0 * (pose_transformed.pose.orientation.w * pose_transformed.pose.orientation.z),
        //           1 - (2 * (pose_transformed.pose.orientation.z * pose_transformed.pose.orientation.z))));

        // transform mtt to /vehicle_odometry
        pcl::PointCloud<pcl::PointXYZ> pct;
        mtt::TargetListPC msg_transformed;

        pcl::PointCloud<pcl::PointXYZ> pct2;
        mtt::TargetListPC msg_transformed2;

        //--------------------------------------------------------------------------------------------------------//
        // ROS_INFO("pc_v1 size = %ld", pc_v1.size());
        // auto before_for3 = std::chrono::high_resolution_clock::now();

        // takes 2 ms max
        for (size_t i = 0; i < pc_v1.size(); ++i) //std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v1 -> contem a nuvem de pontos com as paredes
        {
          if (i == 0)
          {
            try
            {
              // ros::Time time;
              // time.fromNSec(pc_v1[i].header.stamp);
              //Get the transform between two frames by frame ID.
              //pc_v1[i].header.frame_id -> The frame to which data should be transformed
              //"/vehicle_odometry" -> The frame where the data originated
              // ros::Time(0) -> The time at which the value of the transform is desired. (0 will get the latest)
              //transform_mtt -> The transform reference to fill.
              // std::cout << "pc_v1[i].header.frame_id: " << pc_v1[i].header.frame_id << std::endl;

              p_listener->lookupTransform(pc_v1[i].header.frame_id, car_name, ros::Time(0), transform_mtt);
              // ROS_INFO("FFFFFFrame_id=%s ",
              // pc_v1[i].header.frame_id.c_str());

              //Send a StampedTransform The stamped data structure includes frame_id, and time, and parent_id already.
              // mtt_broadcaster.sendTransform(tf::StampedTransform(transform_mtt, time, pc_v1[i].header.frame_id, "/vehicle_odometry"));

              ros::spinOnce();
            }
            catch (tf::TransformException ex)
            {
              ROS_INFO("ERRO NO traj3");

              ROS_ERROR("%s", ex.what());
            }
          }

          // ROS_INFO("pc_v1[%ld] frame_id=%s pcp frame_id=%s", i,
          // pc_v1[i].header.frame_id.c_str(),
          //  pct.header.frame_id.c_str());

          //Apply a rigid transform defined by a 3D offset and a quaternion.
          //pc_v1[i] -> cloud_in
          //pct -> cloud_out
          //transform_mtt.inverse() -> a rigid transformation from tf
          pcl_ros::transformPointCloud(pc_v1[i], pct, transform_mtt.inverse()); //transform_mtt -> tf entre "pc_v1[i].header.frame_id" e "/vehicle_odometry"

          // This message holds a collection of N-dimensional points, which may
          //contain additional information such as normals, intensity, etc. The
          //point data is stored as a binary blob, its layout described by the
          //contents of the "fields" array.
          sensor_msgs::PointCloud2 pc_msg;
          pcl::toROSMsg(pct, pc_msg);
          pc_msg.header.frame_id = car_name;
          pc_msg.header.stamp = ros::Time(0);
          msg_transformed.obstacle_lines.push_back(pc_msg); //insere o ponto na mensagem

          // benchmark_fn("set_limits_walls", [&]() { set_limits_walls(msg_transformed); });

          //takes 1 ms max
          set_limits_walls(msg_transformed);
        }

        // auto after_for3 = std::chrono::high_resolution_clock::now();

        // std::chrono::milliseconds duration_for3 = std::chrono::duration_cast<std::chrono::milliseconds>(after_for3 - before_for3);

        // std::cout << " for cicle 3 took " << duration_for3.count() << "ms." << std::endl;

        // manage_vt->set_obstacles(msg_transformed); //envia as paredes como obstaculos

        // -------------------------------------------------------------------------------------------------------------
        // ------------------------------------------------ROAD LINES---------------------------------------------------
        // -------------------------------------------------------------------------------------------------------------
        // if (_simulation_)
        // {
        // ROS_INFO("pc_v2 size = %ld", pc_v2.size());

        bool _LINES_;
        n.getParam("Param/LINES", _LINES_);

        // takes 2 ms max
        for (size_t i = 0; i < pc_v2.size(); ++i) //std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_v2 -> contem a linha central;
        {
          if (i == 0)
          {
            try
            {
              // ros::Time time;
              // time.fromNSec(pc_v2[i].header.stamp);

              //Get the transform between two frames by frame ID.
              //pc_v2[i].header.frame_id -> The frame to which data should be transformed
              //"/vehicle_odometry" -> The frame where the data originated
              // ros::Time(0) -> The time at which the value of the transform is desired. (0 will get the latest)
              //transform_mtt -> The transform reference to fill.
              // std::cout << "pc_v2[i].header.frame_id: " << pc_v2[i].header.frame_id << std::endl;

              p_listener->lookupTransform(pc_v2[i].header.frame_id, car_name, ros::Time(0), transform_mtt);

              //Send a StampedTransform The stamped data structure includes frame_id, and time, and parent_id already.
              // mtt_broadcaster.sendTransform(tf::StampedTransform(transform_mtt, time, pc_v2[i].header.frame_id, "/vehicle_odometry"));

              ros::spinOnce();
            }
            catch (tf::TransformException ex)
            {
              ROS_INFO("ERRO NO traj1");
              ROS_ERROR("%s", ex.what());
            }
          }
          //Apply a rigid transform defined by a 3D offset and a quaternion.
          //pc_v2[i] -> cloud_in
          //pct2 -> cloud_out
          //transform_mtt.inverse() -> a rigid transformation from tf
          pcl_ros::transformPointCloud(pc_v2[i], pct2, transform_mtt.inverse()); //transform_mtt -> tf entre "pc_v2[i].header.frame_id" e "/vehicle_odometry"

          // This message holds a collection of N-dimensional points, which may
          //contain additional information such as normals, intensity, etc. The
          //point data is stored as a binary blob, its layout described by the
          //contents of the "fields" array.
          sensor_msgs::PointCloud2 pc_msg2;

          pcl::toROSMsg(pct2, pc_msg2);
          pc_msg2.header.frame_id = car_name;
          pc_msg2.header.stamp = ros::Time(0);

          if (_LINES_)
          {
            msg_transformed.obstacle_lines.push_back(pc_msg2); // adiciona a linha como obstaculo
          }

          // msg_transformed.obstacle_lines.push_back(pc_msg2);
          msg_transformed2.obstacle_lines.push_back(pc_msg2);
          set_limits_line(msg_transformed2);
        }

        manage_vt->set_obstacles(msg_transformed); //envia as paredes e outros obstaculos (linha caso _LINES_ seja true) como obstaculos
        manage_vt->set_lines(msg_transformed2);    //ja nao faz nada
        // }

        //   ___________________________________
        //   |                                 |
        //   |        Draw                     |
        //   |_________________________________|
        manage_vt->create_static_markers();

        ros::Time st = ros::Time(0);

        // auto before = std::chrono::high_resolution_clock::now();
        manage_vt->compute_trajectories_scores();

        // benchmark_fn("compute_trajectories_scores", []() { manage_vt->compute_trajectories_scores(); });

        // auto after = std::chrono::high_resolution_clock::now();

        // std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);

        // std::cout << "compute_trajectories_scores: time = " << duration.count() << std::endl;

        // change speed in function of steering angle
        if (manage_vt->chosen_traj.index != -1)
        {
          bool Vel_Ang;
          n.getParam("Param/Vel_Ang", Vel_Ang);

          if (Vel_Ang == true)
          {
            speed = angle_to_speed(manage_vt->chosen_traj.alpha);
          }
          else
          {
            double SPEED_REQUIRED;
            n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
            speed = SPEED_REQUIRED;
          }

          // ROS_INFO("speeeeeeeeeeeeed = %f", speed);
        }

        //   ___________________________________
        //   |                                 |
        //   |     Publish direction and speed |
        //   |_________________________________|
        geometry_msgs::Twist twist_msg;

        if (manage_vt->chosen_traj.index != -1)
        {
          // twist_msg.angular.z =
          // compute_last_dir(manage_vt->chosen_traj.alpha);
          twist_msg.angular.z = compute_last_dir(manage_vt->chosen_traj.alpha);
          if (manage_vt->chosen_traj.score <= 0)
          {
            twist_msg.linear.x = 0;
          }
          else
          {
            twist_msg.linear.x = speed;
          }
          twist_pub.publish(twist_msg);
          // cout << "Message cmd_vel sent" << endl;
        }

        //   ___________________________________
        //   |                                 |
        //   |     Publish distance            |
        //   |_________________________________|

        // std_msgs::Float64MultiArray mindist_msg;
        // mindist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        // mindist_msg.layout.dim[0].label = "sim";
        // mindist_msg.layout.dim[0].size = 2;
        // mindist_msg.layout.dim[0].stride = 2;
        // mindist_msg.layout.data_offset = 0;

        // if (manage_vt->chosen_traj.index != -1)
        // {
        //   double alpha_degrees = manage_vt->chosen_traj.alpha * (180 / M_PI);
        //   // vector<double> vec1 = {manage_vt->chosen_traj.min_dist, alpha_degrees, manage_vt->chosen_traj.index, speed, max_angle};
        //   vector<double> vec1 = {max_angle};
        //   mindist_msg.data.clear();
        //   mindist_msg.data.insert(mindist_msg.data.end(), vec1.begin(), vec1.end());
        //   // mindist_msg.data.insert(chosen_traj.alpha);
        //   mindist_pub.publish(mindist_msg);
        // }

        Publish_traj_data(manage_vt);

        Publish_ap_data(manage_vt);

        Publish_analysis_data(manage_vt);

        //   ___________________________________
        //   |                                 |
        //   |     """"""""""""""""""""""""""" |
        //   |_________________________________|

        // cout<<"Compute scores: "<<(ros::Time::now()-st).toSec();

        // ROS_INFO("manage_vt chosen traj= %f", manage_vt->chosen_traj.alpha);
        // ROS_INFO("chosen traj min dist= %f", manage_vt->chosen_traj.min_dist);

        // trajectory_planner::traj_info info;

        // manage_vt->get_traj_info_msg_from_chosen(&info);

        // commandPublisher.publish(info);

        visualization_msgs::MarkerArray marker_array;
        manage_vt->compute_vis_marker_array(&marker_array);
        array_pub.publish(marker_array);
        // cout<<"ja size:"<<marker_array.markers.size()<<endl;

        have_plan = true;
      }
    }

    // auto after_traj = std::chrono::high_resolution_clock::now();

    // std::chrono::milliseconds duration_traj = std::chrono::duration_cast<std::chrono::milliseconds>(after_traj - before_traj);

    // std::cout << "----- traj took " << duration_traj.count() << "ms." << std::endl;

    loop_rate.sleep();
    ros::spinOnce();
  }
} // main
#endif