/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro -
http://lars.mec.ua.pt All rights reserved.
 Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer. *Redistributions in binary form
must reproduce the above copyright notice, this list of conditions and the
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

/**
 * @file c_manage_trajectory.cpp
 * @brief c_manage_trajectory class manager
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */

/**
 * @file c_manage_trajectory.cpp
 * @brief c_manage_trajectory class manager
 * @author Ricardo Silva
 * @version v1
 * @date 2018-06-06
 */

/**
 * @file c_manage_trajectory.cpp
 * @brief c_manage_trajectory class manager
 * @author Manuel Ferreira
 * @version v2
 * @date 2019-07-03
 */

#include <trajectory_planner/c_manage_trajectory.h>
#include <trajectory_planner/trajectory_planner_nodelet.h>

void CheckOvertaking(std::vector<t_obstacle> &vo);
void CheckSituation(std::vector<t_obstacle> &vo);

double y_max_w_right; //wall
double y_max_w_left;  //wall
double y_min_w_right; //wall
double y_min_w_left;  //wall

double y_min_l_right; //line
double y_min_l_left;  //line
double y_max_l_right; //line
double y_max_l_left;  //line

double WaitingTime;

int count_move_to_left = 0;
int count_stabilize = 0;

// bool still_overtaking = false;

int overtaking_phase = 0;
int detection_times = 0;

double pos_clp_x;
double pos_clp_y;

bool begin_time = true;
bool end_time = true;

ros::Time begin_overtaking;

ros::Duration time_overtaking;

ros::Time begin_3;

/**
 * @brief check the current situation (overtaking, etc.)
 * 
 * @param vo -> obstacles from the walls
 */
void CheckSituation_2try(std::vector<t_obstacle> &vo)
{

  ros::NodeHandle n;
  double DetectDist = 0;
  n.getParam("Param/DetectDist", DetectDist);

  bool OVERTAKING;
  n.getParam("Param/OVERTAKING", OVERTAKING);

  bool DETECTION;
  n.getParam("Param/DETECTION", DETECTION);

  bool CRUZAMENTO;
  n.getParam("Param/CRUZAMENTO", CRUZAMENTO);

  bool DETECTION_BACK;
  n.getParam("Param/DETECTION_BACK", DETECTION_BACK);

  double Detection_Sensitivity;
  n.getParam("Param/Detection_Sensitivity", Detection_Sensitivity);

  double W_DAP_prev;
  n.getParam("Param/W_DAP", W_DAP_prev);

  double W_DLO_prev;
  n.getParam("Param/W_DLO", W_DLO_prev);

  double DETECT_SPACE_SENSIVITY;
  n.getParam("Param/DETECT_SPACE_SENSIVITY", DETECT_SPACE_SENSIVITY);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  double limit_left;
  double limit_right;

  double limit_left_back;
  double limit_right_back;
  int count_points_detected = 0;
  int count_points_detected_front = 0;
  int count_points_detected_back = 0;

  int detect_front = 0;
  int detect_back = 0;

  //----------------------------------------------------------------------------------
  //----------------------------------CRUZAMENTO--------------------------------------
  //----------------------------------------------------------------------------------

  if (CRUZAMENTO == true)
  {
    int count_cruz = 0;
    pcl::PointCloud<pcl::PointXYZRGBA> points_detected_cruzamento;
    for (size_t o = 0; o < vo.size(); ++o)
    {
      // cycle all lines inside each obstacle
      for (size_t lo = 0; lo < vo[o].x.size(); ++lo)
      {

        //CRUZAMENTO
        if (vo[o].y[lo] > pos_clp_y + 1 &&
            vo[o].y[lo] < pos_clp_y + 4) // via tem 5.3 metros
        {
          // t_obstacle o;

          pcl::PointXYZRGBA new_point;
          new_point.x = vo[o].x[lo];
          new_point.y = vo[o].y[lo];
          new_point.z = 0;

          new_point.r = 100;
          new_point.g = 100;
          new_point.b = 100;
          new_point.a = 1;

          count_cruz++;

          points_detected_cruzamento.push_back(new_point);
        }
      }
    }
    // ROS_INFO("detected points: %d", count_cruz);

    PublishColl(points_detected_cruzamento);
  }

  //----------------------------------------------------------------------------------
  //----------------------------------ULTRAPASSAGEM-----------------------------------
  //----------------------------------------------------------------------------------

  double min_dist_front = 500;
  double min_dist_back = 500;

  if (DETECTION == true)
  {

    detect_front = 1;
    if (y_min_l_left < y_min_l_right)
    {
      limit_left = y_min_l_left - DETECT_SPACE_SENSIVITY;
      // ROS_INFO("using limit_left: y_min_l_left");
    }
    else
    {
      limit_left = y_min_l_right - DETECT_SPACE_SENSIVITY;
      // ROS_INFO("using limit_left: y_min_l_right");
    }

    if (y_max_w_right > y_max_w_left)
    {
      limit_right = y_max_w_right + DETECT_SPACE_SENSIVITY;
      // ROS_INFO("using limit_right: y_max_w_right");
      // ROS_INFO("y_max_w_right: %f", y_max_w_right);
      // ROS_INFO("y_max_w_left: %f", y_max_w_left);
    }
    else
    {
      limit_right = y_max_w_left + DETECT_SPACE_SENSIVITY;
      // ROS_INFO("using limit_right: y_max_w_left");
      // ROS_INFO("y_max_w_right: %f", y_max_w_right);
      // ROS_INFO("y_max_w_left: %f", y_max_w_left);
    }

    //EVITAR ERROS DO LIDAR
    if (limit_right < -100 || limit_left > 100)
    {
      limit_right = 0;
      limit_left = 0;
    }

    // ROS_INFO("using limit_left: y_min_l_left");

    PublishCollSpace(limit_left, limit_right, DetectDist);

    pcl::PointCloud<pcl::PointXYZRGBA> points_detected_2;

    for (size_t o = 0; o < vo.size(); ++o)
    {
      // cycle all lines inside each obstacle
      for (size_t lo = 0; lo < vo[o].x.size(); ++lo)
      {

        if (vo[o].x[lo] > 0 &&
            vo[o].x[lo] < DetectDist &&
            vo[o].y[lo] < limit_left - 0.01 &&
            vo[o].y[lo] > limit_right + 0.01)
        {

          // t_obstacle o;

          pcl::PointXYZRGBA new_point;
          new_point.x = vo[o].x[lo];
          new_point.y = vo[o].y[lo];
          new_point.z = 0;

          double dist = sqrt(pow(new_point.x, 2) + pow(new_point.y, 2));

          if (dist < min_dist_front)
          {
            min_dist_front = dist;
          }

          new_point.r = 100;
          new_point.g = 100;
          new_point.b = 100;
          new_point.a = 1;

          points_detected_2.push_back(new_point);

          count_points_detected++;
          count_points_detected_front++;
        }
      }
    }

    // ROS_INFO("points_detected_2: %f", points_detected_2.points);

    PublishColl(points_detected_2);
  }

  if (DETECTION_BACK == true)
  {
    detect_back = 1;
    if (y_max_l_left < y_max_l_right)
    {
      limit_left_back = y_max_l_left;
      // ROS_INFO("using limit_left_back: y_max_l_left");
    }
    else
    {
      limit_left_back = y_max_l_right;
      // ROS_INFO("using limit_left_back: y_max_l_right");
    }

    if (y_min_w_right > y_min_w_left)
    {
      limit_right_back = y_min_w_right;
      // ROS_INFO("using limit_right_back: y_min_w_right");
    }
    else
    {
      limit_right_back = y_min_w_left;
      // ROS_INFO("using limit_right_back: y_min_w_left");
    }

    //EVITAR ERROS DO LIDAR
    if (limit_right_back < -100 || limit_left_back > 100)
    {
      limit_right_back = 0;
      limit_left_back = 0;
    }

    PublishCollSpace_BACK(limit_left_back, limit_right_back, DetectDist);

    pcl::PointCloud<pcl::PointXYZRGBA> points_detected_3;

    for (size_t o = 0; o < vo.size(); ++o)
    {
      // cycle all lines inside each obstacle
      for (size_t lo = 0; lo < vo[o].x.size(); ++lo)
      {

        if (vo[o].x[lo] < 0 && vo[o].x[lo] > (-DetectDist) && vo[o].y[lo] < limit_left_back - 0.01 && vo[o].y[lo] > limit_right_back + 0.01)
        {

          // t_obstacle o;

          pcl::PointXYZRGBA new_point;
          new_point.x = vo[o].x[lo];
          new_point.y = vo[o].y[lo];
          new_point.z = 0;

          double dist = sqrt(pow(new_point.x, 2) + pow(new_point.y, 2));

          if (dist < min_dist_back)
          {
            min_dist_back = dist;
          }

          new_point.r = 100;
          new_point.g = 100;
          new_point.b = 100;
          new_point.a = 1;

          points_detected_3.push_back(new_point);

          count_points_detected++;
          count_points_detected_back++;
        }
      }
    }

    PublishColl_BACK(points_detected_3);
  }

  Publish_DS_data(count_points_detected_front, count_points_detected_back, limit_left, limit_right, limit_left_back, limit_right_back, detect_front, detect_back, min_dist_front, min_dist_back);

  if (OVERTAKING == false)
  {

    n.setParam("Param/AP_right", true);
    n.setParam("Param/AP_left", false);

    if (count_points_detected > Detection_Sensitivity)
    {
      ROS_INFO("detection_times= %d", detection_times);
      if (detection_times < Detection_Sensitivity)
      {
        detection_times++;
      }
      else
      {
        detection_times = 0;
        ROS_INFO("detection_times= %d", detection_times);
        n.setParam("Param/LINES", false);
        n.setParam("Param/OVERTAKING", true);
        // n.setParam("Param/DETECTION", false);
        n.setParam("Param/AP_right", false);
        n.setParam("Param/AP_left", false);
        // n.setParam("Param/Vel_Ang", false);
        // still_overtaking = true;
        overtaking_phase = 1;
        begin_overtaking = ros::Time::now();

        // PUBLISH INFO ----------------------------------------

        ROS_INFO("count_points_detected= %d", count_points_detected);
      }
    }
  }
  else
  {
    double dist_clp = sqrt(pow(pos_clp_x, 2) + pow(pos_clp_y, 2));

    // ROS_INFO("dist_clp: %f", dist_clp);

    if (dist_clp < 1 && overtaking_phase == 1) // começou a curvar e perto da linha
    {
      n.setParam("Param/AP_left", true);
      overtaking_phase = 2;
      ROS_INFO("---------overtaking_phase: 2");
    }

    if (dist_clp > 1 && pos_clp_y < 0 && overtaking_phase == 2) // o veiculo esta a esquerda da linha
    {
      n.setParam("Param/LINES", true);
      n.setParam("Param/DETECTION_BACK", true);
      overtaking_phase = 3;
      ROS_INFO("-------overtaking_phase: 3");
    }

    if (overtaking_phase == 3)
    {
      if (count_points_detected < Detection_Sensitivity)
      {
        ros::Time currtime = ros::Time::now();

        if (begin_time == true)
        {
          ROS_INFO("created timer");
          double SPEED_REQUIRED;
          double SPEED_SAFFETY;
          n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
          n.getParam("Param/SPEED_SAFFETY", SPEED_SAFFETY);

          n.getParam("Param/WaitingTime", WaitingTime);
          WaitingTime = WaitingTime * (SPEED_SAFFETY / SPEED_REQUIRED);
          std::cout << "WaitingTime: " << WaitingTime << std::endl;
          begin_3 = ros::Time::now();
          std::cout << "begin_3: " << begin_3 << std::endl;
          begin_time = false;
        }
        else if (currtime > begin_3 + ros::Duration(WaitingTime))
        {
          //waiting_time
          overtaking_phase = 4;
          n.setParam("Param/DETECTION", false);
          PublishCollSpace(0.0, 0.0, 0.0);
          pcl::PointCloud<pcl::PointXYZRGBA> points_detected_empty;
          PublishColl(points_detected_empty);
          // std::cout << "begin_3: " << begin_3 << std::endl;
          // std::cout << "(currtime - begin_3): " << (currtime - begin_3) << std::endl;
          ROS_INFO("-------------overtaking_phase: 4");
          // ROS_INFO("time: %f", begin_3);
        }
      }
      else
      {
        // ROS_INFO("detecting: %d", count_points_detected);
        begin_time = true;
      }
    }

    if (overtaking_phase == 4)
    {

      if (count_points_detected < Detection_Sensitivity)
      {
        n.setParam("Param/LINES", false);
        n.setParam("Param/AP_left", false);
        n.setParam("Param/AP_right", false);

        if (dist_clp < 1) // começou a curvar e perto da linha
        {
          n.setParam("Param/AP_right", true);
          n.setParam("Param/DETECTION_BACK", false);
          PublishCollSpace_BACK(0.0, 0.0, 0.0);
          pcl::PointCloud<pcl::PointXYZRGBA> points_detected_empty;
          PublishColl_BACK(points_detected_empty);
          n.setParam("Param/W_DAP", 1);
          n.setParam("Param/W_DLO", 0.1);
          overtaking_phase = 5;
          ROS_INFO("---------overtaking_phase: 5");
        }
      }
      else
      {
        n.setParam("Param/LINES", true);
        n.setParam("Param/AP_left", true);
        n.setParam("Param/AP_right", false);

        overtaking_phase = 3;
        ROS_INFO("Obstacle Detected");
        ROS_INFO("---------------overtaking_phase: 3");
      }
    }

    if (overtaking_phase == 5)
    {
      ROS_INFO("dist_clp: %f", dist_clp);
      if (dist_clp > 1 && pos_clp_y > 0) // o veiculo esta a direita da linha
      {
        ros::Time currtime = ros::Time::now();
        // n.setParam("Param/DETECTION", true);
        if (end_time == true)
        {
          ROS_INFO("created timer");
          n.setParam("Param/W_DAP", W_DAP_prev);
          n.setParam("Param/W_DLO", W_DLO_prev);
          n.setParam("Param/LINES", true);
          begin_3 = ros::Time::now();
          std::cout << "begin_3: " << begin_3 << std::endl;
          end_time = false;
        }
        else if (currtime > begin_3 + ros::Duration(WaitingTime))
        {
          n.setParam("Param/OVERTAKING", false);
          n.setParam("Param/Vel_Ang", true);
          ROS_INFO("!!!!!OVERTAKING DONE!!!!!!");
          // n.setParam("Param/DETECTION", true);
          time_overtaking = ros::Time::now() - begin_overtaking;
          std::cout << "time taken to overtake: " << time_overtaking << std::endl;
        }
      }

      // ROS_INFO("FS = %lf",trajectory->score.FS);
      // }
    }
  }
}

/**
 * @brief Extract position of the closest line points of the vehicle
 * 
 * @param msg -> message received from APgenerator
 */
void ExtractCLP2(trajectory_planner::coordinates msg)
{
  pos_clp_x = msg.x;
  pos_clp_y = msg.y;

  //check

  // ROS_INFO("x: %f, y: %f", pos_clp_x, pos_clp_y);
}

/**
 * @brief manually create overtaking (NOT FINISHED!!!!!!!!!!)
 * 
 * @param vo -> obstacles from the walls
 * @param vl -> obstacles from the line
 */
void CheckSituation_done_manually(std::vector<t_obstacle> &vo, std::vector<t_obstacle> &vl)
{
  ros::NodeHandle n;

  bool MANUAL_OVERTAKING;
  n.getParam("Param/MANUAL_OVERTAKING", MANUAL_OVERTAKING);

  if (overtaking_phase == 0)
  {
    n.setParam("Param/LINES", false);
    n.setParam("Param/AP_right", false);
    n.setParam("Param/AP_left", false);

    overtaking_phase = 1;

    ROS_INFO("overtaking_phase: 1");
  }

  int car_number = 0;
  n.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  double dist_clp = sqrt(pow(pos_clp_x, 2) + pow(pos_clp_y, 2));

  // ROS_INFO("dist_clp: %f", dist_clp);

  if (dist_clp < 1 && overtaking_phase == 1) // começou a curvar e perto da linha
  {
    n.setParam("Param/AP_left", true);
    overtaking_phase = 2;
    ROS_INFO("overtaking_phase: 2");
  }

  // ROS_INFO("Analisando a linha");
  if (dist_clp > 1 && pos_clp_y < 0 && overtaking_phase == 2) // o veiculo esta a esquerda da linha
  {
    n.setParam("Param/LINES", true);
    overtaking_phase = 3;
    ROS_INFO("overtaking_phase: 3");
  }

  if (MANUAL_OVERTAKING == false)
  {

    if (abs(pos_clp_x) < 0.2 && overtaking_phase == 3) //veiculo está paralelo à via
    {

      n.setParam("Param/LINES", false);
      n.setParam("Param/AP_left", false);
      n.setParam("Param/AP_right", false);
      overtaking_phase = 4;
      ROS_INFO("overtaking_phase: 4");
    }

    if (dist_clp < 1 && overtaking_phase == 4) // veiculo inclinado e perto da linha
    {
      n.setParam("Param/AP_right", true);
      overtaking_phase = 5;
      ROS_INFO("overtaking_phase: 5");
    }

    if (pos_clp_y > 0 && overtaking_phase == 5)
    {
      overtaking_phase = 6;
      ROS_INFO("overtaking_phase: 6");
    }
  }
}

/**
 * @brief Compute the free space. Determines the inclusion of a point in the
 * rectangle that represents the car geometry.
 * @param trajectory_planner
 * @param vo
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_DLO(c_trajectoryPtr &trajectory, std::vector<t_obstacle> &vo, double DLO_Max, double DetectDist)
{

  // delete all previous computed collision pts
  trajectory->collision_pts.erase(trajectory->collision_pts.begin(), trajectory->collision_pts.end());

  if (trajectory->closest_node < 0 || trajectory->closest_node >= (int)trajectory->x.size()) //nó não existe
  {
    ROS_ERROR("Error on node");
    return FAILURE;
  }

  trajectory->score.DLO = DLO_Max; // Equal to DLO_Max
  trajectory->score.FS = 1;
  trajectory->score.EVAL = 10.0;

  for (int n = 0; n <= trajectory->closest_node; ++n) // cycle all nodes until the closest node
  {
    if (trajectory->v_lines.size() - 1 != trajectory->x.size())
    {
      ROS_ERROR("Node lines and number of nodes not equal");
    }

    // polygon to calculate intersections
    geometry_msgs::Polygon car_polygon;
    for (size_t l = 0; l < trajectory->v_lines[n].size(); ++l)
    {
      geometry_msgs::Point32 point;
      point.x = trajectory->v_lines[n][l].x[0];
      point.y = trajectory->v_lines[n][l].y[0];
      point.z = 0; // pointPcl.z;
      car_polygon.points.push_back(point);
      if (l == (trajectory->v_lines[n].size() - 1))
      {
        point.x = trajectory->v_lines[n][l].x[1];
        point.y = trajectory->v_lines[n][l].y[1];
        point.z = 0; // pointPcl.z;
        car_polygon.points.push_back(point);
      }
    }
    int car_polygon_size = car_polygon.points.size() - 1;
    // ROS_INFO("polygon size = %ld", car_polygon.points.size());

    // cycle all vehicle lines
    for (size_t l = 0; l < trajectory->v_lines[n].size(); ++l)
    {
      double Ax = trajectory->v_lines[n][l].x[0];
      double Ay = trajectory->v_lines[n][l].y[0];
      double Bx = trajectory->v_lines[n][l].x[1];
      double By = trajectory->v_lines[n][l].y[1];

      // cycle all obstacles (walls)
      // ROS_INFO("vo size = %ld", vo.size());
      for (size_t o = 0; o < vo.size(); ++o)
      {
        // cycle all lines inside each obstacle
        for (size_t lo = 0; lo < vo[o].x.size(); ++lo)
        {
          // ROS_INFO("vo[o].x[lo]: %f,vo[o].y[lo]: %f",vo[o].x[lo],vo[o].y[lo]);
          double DLOprev = sqrt(pow(trajectory->v_lines[n][l].x[0] - vo[o].x[lo], 2) + pow(trajectory->v_lines[n][l].y[0] - vo[o].y[lo], 2));

          if (trajectory->score.DLO > DLOprev)
          {
            trajectory->score.DLO = DLOprev;
          }
          // simulator evaluation
          if (n == 0 && trajectory->score.EVAL > DLOprev)
          {
            trajectory->score.EVAL = DLOprev;
          }

          geometry_msgs::Point32 P;
          P.x = vo[o].x[lo];
          P.y = vo[o].y[lo];
          P.z = 0; // pointPcl.z;

          int wn = wn_PnPoly(P, &car_polygon, car_polygon_size);

          if (wn != 0)
          {
            t_point p;
            p.x = P.x;
            p.y = P.y;
            trajectory->collision_pts.push_back(p);
            trajectory->score.FS *= 0;
            // ROS_INFO("FS = %lf",trajectory->score.FS);
          }
        }
      }
    }
  }
  // if (DETECTION == true)
  // {
  //   PublishColl(points_detected_2);
  // }
  return SUCCESS;
}

/**
 * @brief Winding number test for a point in a polygon
 * @param P = a point
 * @param V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
 * @param n = number of points
 * @return wn = the winding number (=0 only when P is outside)
 */
int c_manage_trajectory::wn_PnPoly(geometry_msgs::Point32 P,
                                   geometry_msgs::Polygon *V, int n)
{
  int wn = 0; // the  winding number counter

  // loop through all edges of the polygon
  for (int i = 0; i < n; i++)
  { // edge from V[i] to  V[i+1]
    if (V->points[i].y <= P.y)
    {                                                      // start y <= P.y
      if (V->points[i + 1].y > P.y)                        // an upward crossing
        if (isLeft(V->points[i], V->points[i + 1], P) > 0) // P left of  edge
          ++wn;                                            // have  a valid up intersect
    }
    else
    {                                                      // start y > P.y (no test needed)
      if (V->points[i + 1].y <= P.y)                       // a downward crossing
        if (isLeft(V->points[i], V->points[i + 1], P) < 0) // P right of  edge
          --wn;                                            // have  a valid down intersect
    }
  }
  return wn;
}

/**
 * @brief Tests if a point is Left|On|Right of an infinite line
 * @param P0, P1, and P2 = points
 * @return >0 for P2 left of the line through P0 and P1
           =0 for P2  on the line
           <0 for P2  right of the line
 */
inline int c_manage_trajectory::isLeft(geometry_msgs::Point32 P0,
                                       geometry_msgs::Point32 P1,
                                       geometry_msgs::Point32 P2)
{
  return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
}

/**
 * @brief Verifies intersections between lines
 *
 * @param Ax
 * @param Ay
 * @param Bx
 * @param By
 * @param Cx
 * @param Cy
 * @param Dx
 * @param Dy
 * @param X
 * @param Y
 * @return int
 */
int c_manage_trajectory::lineSegmentIntersection(double Ax, double Ay,
                                                 double Bx, double By,
                                                 double Cx, double Cy,
                                                 double Dx, double Dy,
                                                 double *X, double *Y)
{
  double distAB, theCos, theSin, newX, ABpos;

  //  Fail if either line segment is zero-length.
  if (((Ax == Bx) && (Ay == By)) || ((Cx == Dx) && (Cy == Dy)))
    return DONT_INTERSECT;

  //  Fail if the segments share an end-point.
  if (((Ax == Cx) && (Ay == Cy)) || ((Bx == Cx) && (By == Cy)) ||
      ((Ax == Dx) && (Ay == Dy)) || ((Bx == Dx) && (By == Dy)))
  {
    return DONT_INTERSECT;
  }

  //  (1) Translate the system so that point A is on the origin.
  Bx -= Ax;
  By -= Ay;
  Cx -= Ax;
  Cy -= Ay;
  Dx -= Ax;
  Dy -= Ay;

  //  Discover the length of segment A-B.
  distAB = sqrt(Bx * Bx + By * By);

  //  (2) Rotate the system so that point B is on the positive X axis.
  theCos = Bx / distAB;
  theSin = By / distAB;
  newX = Cx * theCos + Cy * theSin;
  Cy = Cy * theCos - Cx * theSin;
  Cx = newX;
  newX = Dx * theCos + Dy * theSin;
  Dy = Dy * theCos - Dx * theSin;
  Dx = newX;

  //  Fail if segment C-D doesn't cross line A-B.
  if ((Cy < 0. && Dy < 0.) || (Cy >= 0. && Dy >= 0.))
    return DONT_INTERSECT;

  //  (3) Discover the position of the intersection point along line A-B.
  ABpos = Dx + (Cx - Dx) * Dy / (Dy - Cy);

  //  Fail if segment C-D crosses line A-B outside of segment A-B.
  if (ABpos < 0. || ABpos > distAB)
    return DONT_INTERSECT;

  //  (4) Apply the discovered position to line A-B in the original coordinate
  //  system.
  *X = Ax + ABpos * theCos;
  *Y = Ay + ABpos * theSin;

  //  Success.
  return DO_INTERSECT;
}

void set_limits_walls(mtt::TargetListPC &msg)
{
  y_max_w_right = -500;
  y_max_w_left = -500;

  y_min_w_right = -500;
  y_min_w_left = -500;
  double dist_min_reached_detect = 100;

  double dist_y_right;
  double dist_y_right_back;
  double dist_y_right_front;
  if (pos_clp_y < 0)
  {
    dist_y_right_back = abs(pos_clp_y) + 5;
    dist_y_right_front = abs(pos_clp_y) + 5;
  }
  else
  {
    dist_y_right_back = 5 - pos_clp_y;
    dist_y_right_front = 5 - pos_clp_y;
  }

  ros::NodeHandle n;
  double DetectDist = 0;
  n.getParam("Param/DetectDist", DetectDist);

  double DETECT_SPACE_Dist = 0;
  n.getParam("Param/DETECT_SPACE_Dist", DETECT_SPACE_Dist);

  bool first_point_w_right = true;
  bool first_point_w_right_back = true;

  // ROS_INFO("msg_obstacles size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i) //msg.obstacle_lines -> msg_transformed.obstacle_lines -> contem a nuvem de pontos
  {

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc); //nuvem de pontos dos obstaculos

    for (size_t j = 0; j < pc.points.size(); ++j) //para cada ponto
    {
      if (pc.points[j].y > y_max_w_right && pc.points[j].x > 0 && pc.points[j].x < (DetectDist / 10) && pc.points[j].y < -(dist_y_right_front - 1))
      {
        // ROS_INFO("xr: %f, yr: %f", pc.points[j].x, pc.points[j].y);
        y_max_w_right = pc.points[j].y;
      }

      if (pc.points[j].y > y_min_w_right && pc.points[j].x < 0 && pc.points[j].x > (-(DetectDist / 2)) && pc.points[j].y < -(dist_y_right_back - 1))
      {
        // ROS_INFO("------ xl: %f, yl: %f", pc.points[j].x, pc.points[j].y);
        y_min_w_right = pc.points[j].y;
      }

      if (pc.points[j].y > y_max_w_left && pc.points[j].y < 0 && pc.points[j].x > (DetectDist - (DetectDist / 2)) && pc.points[j].x < DetectDist)
      {
        // ROS_INFO("------ xl: %f, yl: %f", pc.points[j].x, pc.points[j].y);
        if (first_point_w_right == true)
        {
          dist_y_right = pc.points[j].y;
          y_max_w_left = pc.points[j].y;
          first_point_w_right = false;
        }
        else
        {
          if (abs(pc.points[j].y - dist_y_right) < DETECT_SPACE_Dist)
          {
            y_max_w_left = pc.points[j].y;
          }
        }
      }

      if (pc.points[j].y > y_min_w_left && pc.points[j].y < 0 && pc.points[j].x > (-DetectDist) && pc.points[j].x < (DetectDist / 10) - DetectDist)
      {
        // ROS_INFO("------ xl: %f, yl: %f", pc.points[j].x, pc.points[j].y);
        if (first_point_w_right_back == true)
        {
          dist_y_right_back = pc.points[j].y;
          y_min_w_left = pc.points[j].y;
          first_point_w_right_back = false;
        }
        else
        {
          if (abs(pc.points[j].y - dist_y_right_back) < DETECT_SPACE_Dist)
          {
            y_min_w_left = pc.points[j].y;
          }
        }
      }
    }
  }
}

void set_limits_line(mtt::TargetListPC &msg)
{

  y_min_l_right = 500;
  y_min_l_left = 500;
  y_max_l_right = 500;
  y_max_l_left = 500;

  ros::NodeHandle n;
  double DetectDist = 0;
  n.getParam("Param/DetectDist", DetectDist);

  // ROS_INFO("msg_lines size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i) //msg.obstacle_lines = msg_transformed2.obstacle_lines = pc_msg2 (pontos?)
  {

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc); //nuvem de pontos dos obstaculos

    for (size_t j = 0; j < pc.points.size(); ++j) //para cada ponto
    {

      if (pc.points[j].y < y_min_l_right && pc.points[j].x > (DetectDist - DetectDist / 10) && pc.points[j].x < DetectDist)
      {
        y_min_l_right = pc.points[j].y;
      }

      if (pc.points[j].y < y_max_l_right && pc.points[j].x < (DetectDist / 10 - DetectDist) && pc.points[j].x > (-DetectDist))
      {
        y_max_l_right = pc.points[j].y;
      }

      if (pc.points[j].y < y_min_l_left && pc.points[j].x > 0)
      {
        y_min_l_left = pc.points[j].y;
      }

      if (pc.points[j].y < y_max_l_left && pc.points[j].x < 0)
      {
        y_max_l_left = pc.points[j].y;
      }
    }

    //EVITAR ERROS DE DETACAO DOS LIDAR
    if (y_min_l_right > 100)
    {
      y_min_l_right = 0;
    }
    if (y_max_l_right > 100)
    {
      y_max_l_right = 0;
    }
    if (y_min_l_left > 100)
    {
      y_min_l_left = 0;
    }
    if (y_max_l_left > 100)
    {
      y_max_l_left = 0;
    }
  }
}

/**
 * @brief Sets the obstacles
 * @param mtt::TargetListPC& msg
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_obstacles(mtt::TargetListPC &msg) //msg -> msg_transformed
{
  vo.erase(vo.begin(), vo.end()); //std::vector<t_obstacle> vo

  // ROS_INFO("msg_obstacles size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i) //msg.obstacle_lines -> msg_transformed.obstacle_lines -> contem a nuvem de pontos
  {
    //typedef struct
    // {
    //   std::vector<double> x;
    //   std::vector<double> y;
    //   int id;
    // } t_obstacle;

    t_obstacle o;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc); //nuvem de pontos dos obstaculos

    for (size_t j = 0; j < pc.points.size(); ++j) //para cada ponto
    {

      o.x.push_back(pc.points[j].x); //obstaculo (o) recebe as posições x e y da nuvem de pontos
      o.y.push_back(pc.points[j].y);
    }

    vo.push_back(o); //vo tem agora a nuvem de pontos
  }
  return SUCCESS;
}

/**
 * @brief Sets the obstacles
 * @param mtt::TargetListPC& msg
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_lines(mtt::TargetListPC &msg)
{
  vl.erase(vl.begin(), vl.end());

  // ROS_INFO("msg_lines size = %ld", msg.obstacle_lines.size());
  for (size_t i = 0; i < msg.obstacle_lines.size(); ++i) //msg.obstacle_lines = msg_transformed2.obstacle_lines = pc_msg2 (pontos?)
  {
    //typedef struct
    // {
    //   std::vector<double> x;
    //   std::vector<double> y;
    //   int id;
    // } t_obstacle;

    t_obstacle o;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg.obstacle_lines[i], pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc); //nuvem de pontos dos obstaculos

    for (size_t j = 0; j < pc.points.size(); ++j) //para cada ponto
    {
      o.x.push_back(pc.points[j].x); //obstaculo (o) recebe as posições x e y da nuvem de pontos
      o.y.push_back(pc.points[j].y);
    }

    vl.push_back(o); //vl tem agora a nuvem de pontos
  }
  return SUCCESS;
}

/**
 * @brief Set the vehicle description
 *
 * @param w
 * @param lb
 * @param lf
 * @param ht
 * @param hb
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_vehicle_description(double w, double lb,
                                                           double lf, double ht,
                                                           double hb)
{
  vehicle_description.width = w;
  vehicle_description.lenght_back = lb;
  vehicle_description.lenght_front = lf;
  vehicle_description.height_top = ht;
  vehicle_description.height_bottom = hb;
  return SUCCESS;
}

/**
 * @brief Gets the info of chosen nodes
 *
 * @param info
 * @return t_func_output
 */
t_func_output c_manage_trajectory::get_traj_info_msg_from_chosen(
    trajectory_planner::traj_info *info)
{
  info->arc.erase(info->arc.begin(), info->arc.end());
  info->total_arc.erase(info->total_arc.begin(), info->total_arc.end());
  info->alpha.erase(info->alpha.begin(), info->alpha.end());
  info->speed.erase(info->speed.begin(), info->speed.end());
  for (int j = 0; j <= (int)vt[chosen_traj.index]->closest_node; ++j)
  {
    info->arc.push_back(vt[chosen_traj.index]->arc[j]);
    info->total_arc.push_back(vt[chosen_traj.index]->total_arc[j]);
    info->alpha.push_back(vt[chosen_traj.index]->alpha[j]);
  }
  set_speed_vector(info);

  // cout<<"ARC SIZE (when constructing) "<<info->arc.size()<<endl;
  // cout<<"TOTAL ARC SIZE (when constructing) "<<info->total_arc.size()<<endl;
  // cout<<"ALPHA SIZE (when constructing) "<<info->alpha.size()<<endl;
  // cout<<"SPEED SIZE (when constructing) "<<info->speed.size()<<endl;
  return SUCCESS;
}

/**
 * @brief Determine the speed vector
 *
 * @param info
 * @return t_func_output
 */
t_func_output
c_manage_trajectory::set_speed_vector(trajectory_planner::traj_info *info)
{

  ros::NodeHandle n;
  double SPEED_REQUIRED;
  double SPEED_SAFFETY;
  n.getParam("Param/SPEED_REQUIRED", SPEED_REQUIRED);
  n.getParam("Param/SPEED_SAFFETY", SPEED_SAFFETY);

  for (int i = 0; i <= (int)vt[chosen_traj.index]->closest_node; ++i)
  {
    if (i < ((int)vt[chosen_traj.index]->arc.size() - 1))
    {
      if ((info->arc[i]) * (info->arc[i + 1]) < 0.0)
      {
        info->speed.push_back(
            (info->arc[i] / fabs(info->arc[i])) *
            SPEED_SAFFETY); // This is the speed set to reverse/forward or
                            // forward/reverse
      }
      else
      {
        info->speed.push_back((info->arc[i] / fabs(info->arc[i])) *
                              SPEED_REQUIRED);
      }
    }
    else
    {
      info->speed.push_back((info->arc[i] / fabs(info->arc[i])) *
                            SPEED_REQUIRED);
    }
  }
  return SUCCESS;
}

/**
 * @brief Set the chosen trajectory
 * @param int n
 * @return int
 */
int c_manage_trajectory::set_chosen_traj(int n)
{
  chosen_traj.index = n;
  // cout << "Chosen trajectory index1: " << chosen_traj.index << endl;
  return 1;
}

/**
 * @brief Set the attractor point
 * @param x
 * @param y
 * @param theta
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_attractor_point(double x, double y,
                                                       double theta)
{
  AP.x = x;
  AP.y = y;
  AP.theta = theta;
  return SUCCESS;
}

/**
 * @brief Computes the trajectory global score
 * @param trajectory
 * @return t_func_output
 */
t_func_output
c_manage_trajectory::compute_global_traj_score(c_trajectoryPtr &trajectory, double W_DAP, double W_ADAP, double W_DLO)
{
  // double W_DAP = 0.40;
  // double W_ADAP = 0.35;
  // double W_DLO = 0.25;

  // ros::NodeHandle n;
  // double W_DAP;
  // n.getParam("Param/W_DAP", W_DAP);
  // double W_ADAP;
  // n.getParam("Param/W_ADAP", W_ADAP);
  // double W_DLO;
  // n.getParam("Param/W_DLO", W_DLO);

  trajectory->score.overall_norm =
      (W_DAP * trajectory->score.DAPnorm + W_ADAP * trajectory->score.ADAPnorm +
       W_DLO * trajectory->score.DLOnorm) *
      trajectory->score.FS;
  // cout<<"Overallscore= "<<trajectory->score.overall_norm<<endl;

  return SUCCESS;
}

/**
 * @brief Set the inter-axis distance of the vehicle
 * @param val
 * @return t_func_output
 */
t_func_output c_manage_trajectory::set_inter_axis_distance(double val)
{
  inter_axis_distance = val;
  return SUCCESS;
}

/**
 * @brief Create a trajectory
 * @param alpha_in
 * @param arc_in
 * @param speed_in
 * @return t_func_output
 */
t_func_output c_manage_trajectory::create_new_trajectory(
    vector<double> alpha_in, vector<double> arc_in, vector<double> speed_in)
{
  // allocate space for new traj
  c_trajectoryPtr t_ptr(new c_trajectory(inter_axis_distance));
  vt.push_back(t_ptr);

  // set the parameters of the traj
  return vt[vt.size() - 1]->generate(alpha_in, arc_in, speed_in,
                                     vehicle_description);
}

/**
 * @brief Update a trajectory during planning execution
 * @param alpha_in
 * @param arc_in
 * @param speed_in
 * @param traj_num
 * @return t_func_output
 */
t_func_output c_manage_trajectory::update_trajectory(vector<double> alpha_in,
                                                     vector<double> arc_in,
                                                     vector<double> speed_in,
                                                     int traj_num)
{
  // set the parameters of the traj
  return vt[traj_num]->generate(alpha_in, arc_in, speed_in,
                                vehicle_description);
}

/**
 * @brief Chooses the trajectory
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_chosen_traj(void)
{
  double max_val = -1000;
  chosen_traj.index = -1;

  for (int i = 0; i < (int)vt.size(); i++)
  {
    if (vt[i]->score.overall_norm > max_val)
    {
      chosen_traj.index = i;
      // cout << "Chosen trajectory index2: " << chosen_traj.index << endl;
      chosen_traj.min_dist = vt[i]->score.EVAL;
      chosen_traj.alpha = vt[i]->alpha[0];
      chosen_traj.score = vt[i]->score.overall_norm;
      max_val = vt[i]->score.overall_norm;
    }
  }

  if (chosen_traj.index != -1)
    return SUCCESS;
  else
    return FAILURE;
}

// double getAlpha (){
//   return chosen_traj.alpha;
// }

/**
 * @brief Create a static marker
 * @return t_func_output
 */
t_func_output c_manage_trajectory::create_static_markers(void)
{
  // cout<<"ja create static"<<endl;
  static_marker_vec.clear();
  int marker_count = 0;
  for (int i = 0; i < (int)vt.size(); i++)
  {
    vt[i]->create_markers(&static_marker_vec, &marker_count, i);
  }
  return SUCCESS;
}

/**
 * @brief Compute the markers array
 * @param  marker_array
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_vis_marker_array(
    visualization_msgs::MarkerArray *marker_array)
{
  std::vector<visualization_msgs::Marker> marker_vec;
  ros::NodeHandle nh;
  int TRAJ_INFO = 1;
  nh.getParam("Param/TRAJ_INFO", TRAJ_INFO);
  // std::cout << "TRAJ_INFO=" << TRAJ_INFO << std::endl;

  // ROS_INFO("static marker vec size=%ld", static_marker_vec.size());
  for (size_t i = 0; i < static_marker_vec.size(); ++i)
  {
    marker_vec.push_back(static_marker_vec[i]);
  }

  int marker_count = 0;
  if (TRAJ_INFO == 0)
  {
    for (int i = 0; i < (int)vt.size(); ++i)
    {
      draw_on_node(vt[i], &marker_vec, &marker_count, 0.60, (vt[i]->score.DAP + vt[i]->score.ADAP + vt[i]->score.DLO) * vt[i]->score.FS, vt[i]->score.overall_norm, "P= ");
    }
  }
  else if (TRAJ_INFO == 1)
  {
    for (int i = 0; i < (int)vt.size(); ++i)
    {
      draw_on_node(vt[i], &marker_vec, &marker_count, 0.15, vt[i]->score.DAP, vt[i]->score.DAPnorm, "DAP= ");
    }
  }
  else if (TRAJ_INFO == 2)
  {
    for (int i = 0; i < (int)vt.size(); ++i)
    {
      draw_on_node(vt[i], &marker_vec, &marker_count, 0.30, vt[i]->score.ADAP, vt[i]->score.ADAPnorm, "ADAP= ");
    }
  }
  else if (TRAJ_INFO == 3)
  {
    for (int i = 0; i < (int)vt.size(); ++i)
    {
      draw_on_node(vt[i], &marker_vec, &marker_count, 2.5, vt[i]->score.DLO, vt[i]->score.DLOnorm, "DLO= ");
    }
  }

  
  int car_number = 0;
  nh.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  // ________________________________
  //|                                |
  //|           Line List            |
  //|________________________________|
  // Line marker to trajectory
  visualization_msgs::Marker marker2;
  geometry_msgs::Point p;
  marker2.header.frame_id = car_name;
  marker2.header.stamp = ros::Time(0);
  marker2.ns = "chosen_trajectory";
  marker2.id = 0;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.type = visualization_msgs::Marker::LINE_LIST;
  marker2.scale.x = 0.08;
  marker2.color.r = 0.039;
  marker2.color.g = 0.619;
  marker2.color.b = 0.039;
  marker2.color.a = 1.00;
  int first_step = 1;
  for (size_t i = 0; i < vt[chosen_traj.index]->x.size(); ++i)
  {
    if (first_step == 1)
    {
      p.x = 0;
      p.y = 0;
      p.z = 2;
      marker2.points.push_back(p);
      p.x = vt[chosen_traj.index]->x[i];
      p.y = vt[chosen_traj.index]->y[i];
      p.z = 2;
      marker2.points.push_back(p);
      first_step = 0;
    }
    else
    {
      p.x = vt[chosen_traj.index]->x[i - 1];
      p.y = vt[chosen_traj.index]->y[i - 1];
      p.z = 2;
      marker2.points.push_back(p);
      p.x = vt[chosen_traj.index]->x[i];
      p.y = vt[chosen_traj.index]->y[i];
      p.z = 2;
      marker2.points.push_back(p);
    }
  }
  marker_vec.push_back(marker2);

  // ________________________________
  //|                                |
  //|     Rectangle (actual)         |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker5;
  marker5.header.frame_id = car_name;
  marker5.header.stamp = ros::Time(0);
  marker5.ns = "car_actual_traj";
  marker5.id = 0;
  marker5.action = visualization_msgs::Marker::ADD;
  marker5.type = visualization_msgs::Marker::CUBE;
  marker5.scale.x = _VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_;
  marker5.scale.y = _VEHICLE_WIDTH_;
  marker5.scale.z = 0.001;
  marker5.color.r = 0;
  marker5.color.g = 1;
  marker5.color.b = 0;
  marker5.color.a = 0.1;
  for (size_t i = 0; i <= vt[chosen_traj.index]->x.size(); ++i)
  {
    if (i == 0)
    {
      marker5.pose.position.x =
          ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
           _VEHICLE_LENGHT_BACK_) *
          cos(0);
      marker5.pose.position.y =
          ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
           _VEHICLE_LENGHT_BACK_) *
          sin(0);
      marker5.pose.position.z = -0.1;
      marker5.pose.orientation.z = sin(0);
      marker5.pose.orientation.w = cos(0);
    }
    else
    {
      marker5.pose.position.x =
          vt[chosen_traj.index]->x[i - 1] +
          ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
           _VEHICLE_LENGHT_BACK_) *
              cos(vt[chosen_traj.index]->theta[i - 1]);
      marker5.pose.position.y =
          vt[chosen_traj.index]->y[i - 1] +
          ((_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
           _VEHICLE_LENGHT_BACK_) *
              sin(vt[chosen_traj.index]->theta[i - 1]);
      marker5.pose.position.z = -0.1;
      marker5.pose.orientation.z = sin(vt[chosen_traj.index]->theta[i - 1] / 2);
      marker5.pose.orientation.w = cos(vt[chosen_traj.index]->theta[i - 1] / 2);
    }
    marker_vec.push_back(marker5);
    marker5.id += 1;
  }

  // ________________________________
  //|                                |
  //|        Best node (sphere)      |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker3;
  marker3.header.frame_id = car_name;
  marker3.header.stamp = ros::Time(0);
  marker3.ns = "closest_node";
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.scale.x = 0.2;
  marker3.scale.y = 0.2;
  marker3.scale.z = 0.2;
  marker3.color.r = 1.0;
  marker3.color.g = 0.0;
  marker3.color.b = 0.1;
  marker3.color.a = 0.6;

  for (size_t i = 0; i < vt.size(); ++i)
  {
    marker3.id += 2;
    marker3.pose.position.x = vt[i]->x[vt[i]->closest_node];
    marker3.pose.position.y = vt[i]->y[vt[i]->closest_node];
    marker3.pose.position.z = 0;
    marker_vec.push_back(marker3);
  }

  // ________________________________
  //|                                |
  //|        Colision (cylinder)     |
  //|________________________________|
  // Represents the form of the car in each node
  visualization_msgs::Marker marker4;
  marker4.header.frame_id = car_name;
  marker4.header.stamp = ros::Time(0);
  marker4.ns = "colision_points";
  marker4.action = visualization_msgs::Marker::ADD;
  marker4.type = visualization_msgs::Marker::CYLINDER;
  marker4.scale.x = 0.12;
  marker4.scale.y = 0.12;
  marker4.scale.z = 0.1;
  marker4.color.r = 1.0;
  marker4.color.g = 0.84;
  marker4.color.b = 0.0;
  marker4.color.a = 1.0;

  static size_t coli_marker_total = 0;
  int total = 0;
  for (size_t j = 0; j < vt.size(); ++j)
  {
    // ROS_INFO("Traj%ld has %ld collisions\n", j, vt[j]->collision_pts.size());

    for (size_t i = 0; i < vt[j]->collision_pts.size(); ++i)
    {
      marker4.id = total;
      marker4.pose.position.x = vt[j]->collision_pts[i].x;
      marker4.pose.position.y = vt[j]->collision_pts[i].y;
      marker_vec.push_back(marker4);
      total++;
    }
  }

  // ROS_INFO("total=%d coli=%ld", total, coli_marker_total);
  // erase old colision markers
  for (size_t j = total; j < coli_marker_total; ++j)
  {
    // ROS_INFO("deleting marker ");
    marker4.header.frame_id = car_name;
    marker4.id = j;
    marker4.action = visualization_msgs::Marker::DELETE;
    marker_vec.push_back(marker4);
  }

  coli_marker_total = total;

  marker_array->markers = marker_vec;

  return SUCCESS;
}

/**
 * @brief Compute trajectories scores
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_trajectories_scores(void)
{
  // double maximum_admissible_to_DAP = 8.0;  // ATENTION to negative values if bigger than maximum
  // double maximum_admissible_to_DLO = 10.0; // ATENTION to negative values if bigger than maximum

  ros::NodeHandle nh;
  double APdistMax;
  nh.getParam("Param/APdistMax", APdistMax);
  double DLO_Max;
  nh.getParam("Param/DLO_Max", DLO_Max);
  bool DETECTION;
  nh.getParam("Param/DETECTION", DETECTION);
  bool OVERTAKING;
  nh.getParam("Param/OVERTAKING", OVERTAKING);
  bool CRUZAMENTO;
  nh.getParam("Param/CRUZAMENTO", CRUZAMENTO);
  bool MANUAL_OVERTAKING;
  nh.getParam("Param/MANUAL_OVERTAKING", MANUAL_OVERTAKING);

  if (DETECTION == true || OVERTAKING == true || CRUZAMENTO == true)
  {
    // benchmark_fn("CheckSituation_2try", [&]() { CheckSituation_2try(vo); });
    CheckSituation_2try(vo); //!!!! Uncomment to detect objects
  }
  else
  {
    PublishCollSpace(0.0, 0.0, 0.0);
    pcl::PointCloud<pcl::PointXYZRGBA> points_detected_empty;
    // PublishColl(points_detected_empty);
  }

  if (MANUAL_OVERTAKING == true)
  {
    CheckSituation_done_manually(vo, vl);
  }

  double DetectDist = 0;
  nh.getParam("Param/DetectDist", DetectDist);

  // auto before_for = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < (int)vt.size(); ++i)
  {

    // CheckSituation(vo); //!!!! Uncomment to detect objects

    // Compute DAP and ADAP
    // benchmark_fn("compute_DAP", [&]() { compute_DAP(vt[i], AP); });

    compute_DAP(vt[i], AP);

    // Compute DLO
    // benchmark_fn("compute_DLO", [&]() { compute_DLO(vt[i], vo); });
    compute_DLO(vt[i], vo, DLO_Max, DetectDist);

    // normalize DAP
    vt[i]->score.DAPnorm = max(0.0, (1 - (vt[i]->score.DAP) / APdistMax));

    // normalize ADAP
    vt[i]->score.ADAPnorm = max(0.0, (1 - (vt[i]->score.ADAP / (M_PI))));

    // normalize DLO
    vt[i]->score.DLOnorm = (vt[i]->score.DLO) / DLO_Max;
  }
  // auto after_for = std::chrono::high_resolution_clock::now();

  // std::chrono::milliseconds duration_for = std::chrono::duration_cast<std::chrono::milliseconds>(after_for - before_for);

  // std::cout << " for cicle took " << duration_for.count() << "ms." << std::endl;

  double W_DAP;
  nh.getParam("Param/W_DAP", W_DAP);
  double W_ADAP;
  nh.getParam("Param/W_ADAP", W_ADAP);
  double W_DLO;
  nh.getParam("Param/W_DLO", W_DLO);

  // compute overall score for each traj
  for (size_t i = 0; i < vt.size(); ++i)
  {
    // benchmark_fn("compute_global_traj_score", [&]() { compute_global_traj_score(vt[i], W_DAP, W_ADAP, W_DLO); });
    compute_global_traj_score(vt[i], W_DAP, W_ADAP, W_DLO);
  }

  // benchmark_fn("compute_chosen_traj", [&]() { compute_chosen_traj(); });

  compute_chosen_traj();

  return SUCCESS;
}

/**
 * @brief Compute the distance to application point
 * @param trajectory
 * @param AP
 * @return t_func_output
 */
t_func_output c_manage_trajectory::compute_DAP(c_trajectoryPtr &trajectory,
                                               t_desired_coordinates &AP)
{
  trajectory->score.DAP = 10e6;

  trajectory->closest_node = -1;

  for (size_t i = 0; i < trajectory->x.size(); ++i) //verifica todos os nós pelo mais proximo
  {
    double DAP_prev = sqrt(pow(trajectory->x[i] - AP.x, 2) + pow(trajectory->y[i] - AP.y, 2)); //distancia do nó ao AP

    if (DAP_prev < trajectory->score.DAP) //se for mais proximo
    {
      trajectory->score.DAP = DAP_prev; //update score
      trajectory->closest_node = i;     //nó mais proximo
    }
  }

  if (trajectory->closest_node != -1)
  {
    trajectory->score.ADAP = compute_ADAP(trajectory, AP, trajectory->closest_node);
    return SUCCESS;
  }
  else
  {
    return FAILURE;
  }
}

/**
 * @brief Compute the angular difference
 * @param trajectory
 * @param AP
 * @param i
 * @return double
 */
double c_manage_trajectory::compute_ADAP(c_trajectoryPtr &trajectory,
                                         t_desired_coordinates &AP, int i)
{
  double ang_AP = atan2(AP.y, AP.x);                           //radians // ângulo do ponto atrator
  double ang_node = atan2(trajectory->y[i], trajectory->x[i]); //radians // ângulo do nó
  // double adap = abs(trajectory->theta[i] - AP.theta);
  double adap = abs(ang_node - ang_AP);
  if (adap > M_PI)
    adap = 2 * M_PI - adap;
  return adap;
}

/**
 * @brief Draw some informations about trajectories
 * @param trajectory_planner
 * @param marker_vec
 * @param marker_count
 * @param z_high
 * @param value
 * @param normalized_value
 * @param string_init
 */
void c_manage_trajectory::draw_on_node(
    c_trajectoryPtr &trajectory,
    std::vector<visualization_msgs::Marker> *marker_vec, int *marker_count,
    double z_high, double value, double normalized_value, string string_init)
{
  // Create a marker
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;

  ros::NodeHandle nh;
  int car_number = 0;
  nh.getParam("car_number", car_number);
  char car_name[20] = "/vehicle_odometry_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(car_name, car_number_string);

  // ________________________________
  //|                                |
  //|           text nodes           |
  //|________________________________|
  // Points marker to t nodes
  marker.header.frame_id = car_name;
  marker.header.stamp = ros::Time(0);
  marker.ns = "info";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.6;
  marker.color.r = 0.039;
  marker.color.g = 0.207;
  marker.color.b = 0.619;
  marker.color.a = 1.0;
  marker.id = (*marker_count)++;
  marker.pose.position.x =
      trajectory->x[trajectory->x.size() - 1] +
      (_VEHICLE_LENGHT_FRONT_ +
       (_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
       _VEHICLE_LENGHT_BACK_) *
          cos(trajectory->theta[trajectory->theta.size() - 1]);
  marker.pose.position.y =
      trajectory->y[trajectory->y.size() - 1] +
      (_VEHICLE_LENGHT_FRONT_ +
       (_VEHICLE_LENGHT_FRONT_ + _VEHICLE_LENGHT_BACK_) / 2 -
       _VEHICLE_LENGHT_BACK_) *
          sin(trajectory->theta[trajectory->theta.size() - 1]);
  marker.pose.position.z = z_high;

  marker.text = string_init + str(boost::format("%.2f") % normalized_value);
  // string_init + str(boost::format("%.2f") % value) + " (" +
  // str(boost::format("%.2f") % normalized_value) + ")";
  marker_vec->push_back(marker);
}