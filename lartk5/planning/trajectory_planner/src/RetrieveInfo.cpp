#include <fstream>
#include <stdlib.h> /* atof */
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "gazebo_msgs/ModelStates.h"
#include <trajectory_planner/coordinates.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//-----------------------------------

ros::Subscriber min_dist_sub;
ros::Subscriber vel_sub;
ros::Subscriber clp_sub;
ros::Subscriber traj_sub;
ros::Subscriber ap_sub;
ros::Subscriber ap_extract;
ros::Subscriber DS_sub;
ros::Subscriber Coll_points_front_sub;
ros::Subscriber Coll_points_back_sub;

double pos_clp_x;
double pos_clp_y;
int lane;

double ap_x;
double ap_y;
double dist_ap;

int coll_points_front;
int coll_points_back;

int car_number = 0;
char car_number_string[2];

// std::ofstream file_min_dist;
// std::ofstream file_speed;

/**
 * @brief Export Atractor Point Info
 * 
 * @param msg 
 */
void Extract_AP(std_msgs::Float64MultiArray msg)
{

    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);
    bool AP_right;
    bool AP_left;
    n.getParam("Param/AP_right", AP_right);
    n.getParam("Param/AP_left", AP_left);
    int ap_lane;

    if (AP_left == true && AP_right == false)
    {
        ap_lane = 0;
    }
    else if (AP_left == false && AP_right == false)
    {
        ap_lane = 1;
    }
    else if (AP_left == false && AP_right == true)
    {
        ap_lane = 2;
    }

    if (RECORDING == true)
    {
        if (car_number == 0)
        {
            n.getParam("car_number", car_number);
            sprintf(car_number_string, "%d", car_number);
        }
        char path_begin[100] = "/home/ana/catkin_ws/src/Multiple_AV/lartk5/planning/trajectory_planner/src/csv/ap_";
        char path_end[5] = ".csv";
        strcat(path_begin, car_number_string);
        strcat(path_begin, path_end);
        std::ofstream file_ap(path_begin, std::ios_base::app);

        // std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        // 0 -> speed

        file_ap << ros::Time::now() << "," << ap_lane << "," << ap_x << "," << ap_y << "," << dist_ap << "," << msg.data[0] << "\n";
    }
}

/**
 * @brief Export Detected_Space Info
 * 
 * @param msg 
 */
void Extract_DS(std_msgs::Float64MultiArray msg)
{

    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);

    if (RECORDING == true)
    {
        if (car_number == 0)
        {
            n.getParam("car_number", car_number);
            sprintf(car_number_string, "%d", car_number);
        }
        char path_begin[100] = "/home/ana/catkin_ws/src/Multiple_AV/lartk5/planning/trajectory_planner/src/csv/DS_";
        char path_end[5] = ".csv";
        strcat(path_begin, car_number_string);
        strcat(path_begin, path_end);
        std::ofstream file_DS(path_begin, std::ios_base::app);

        // std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        // 0 -> count_points_detected_front,
        // 1 -> count_points_detected_back,
        // 2 -> limit_left,
        // 3 -> limit_right,
        // 4 -> limit_left_back,
        // 5 -> limit_right_back,
        // 6 -> detect_front,
        // 7 -> detect_back
        // 8 -> min_dist_front
        // 9 -> min_dist_back

        file_DS << ros::Time::now() << "," << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "," << msg.data[4] << "," << msg.data[5] << "," << msg.data[6] << "," << msg.data[7] << "," << msg.data[8] << "," << msg.data[9] << "\n";
    }
}

/**
 * @brief Export basic data
 * 
 * @param msg 
 */
void Extract_min_dist(std_msgs::Float64MultiArray msg)
{
    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);
    bool _LINES_;
    n.getParam("Param/LINES", _LINES_);
    
    int lines;

    if (_LINES_ == true)
    {
        lines = 1;
    }
    else
    {
        lines = 0;
    }

    if (RECORDING == true)
    {
        if (car_number == 0)
        {
            n.getParam("car_number", car_number);
            sprintf(car_number_string, "%d", car_number);
        }
        char path_begin[100] = "/home/ana/catkin_ws/src/Multiple_AV/lartk5/planning/trajectory_planner/src/csv/analysis_data_";
        char path_end[5] = ".csv";
        strcat(path_begin, car_number_string);
        strcat(path_begin, path_end);
        std::ofstream file_min_dist(path_begin, std::ios_base::app);

        // 0 -> manage_vt->chosen_traj.min_dist
        // 1 -> manage_vt->chosen_traj.index
        // 2 -> alpha_degrees
        // 3 -> max_angle
        // 4 -> this_speed_new

        // std::cout << ros::Time::now() << ",DLO: " << DLO << std::endl;

        file_min_dist << ros::Time::now() << "," << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "," << msg.data[4] << "," << lane << "," << pos_clp_y << "," << coll_points_front << "," << coll_points_back << "," << lines << "\n";
    }
}

/**
 * @brief Extract current lane and distance to the central line
 * 
 * @param msg 
 */
void ExtractCLP2(trajectory_planner::coordinates msg)
{
    pos_clp_y = msg.y;
    if (pos_clp_y > 0)
    {
        lane = 1; //right lane
    }
    else
    {
        lane = 0; //left lane
    }
    //check

    // ROS_INFO("x: %f, y: %f", pos_clp_x, pos_clp_y);
}

/**
 * @brief Extract Atractor Point info
 * 
 * @param msg 
 */
void get_AP(trajectory_planner::coordinates msg)
{
    ap_x = msg.x;
    ap_y = msg.y;
    dist_ap = sqrt(pow(ap_x, 2) + pow(ap_y, 2));
    // std::cout << "ap_x: " << ap_x << std::endl;
    // ROS_INFO("ap_x: %f", ap_x);
}

/**
 * @brief Extract number of collision points detected from the Front Space Detection
 * 
 * @param msg 
 */
void Extract_collision_points_front(sensor_msgs::PointCloud2 msg)
{
    coll_points_front = msg.width;
    // std::cout << "points cz received: " << cruz_points << std::endl;
}

/**
 * @brief Extract number of collision points detected from the Back Space Detection
 * 
 * @param msg 
 */
void Extract_collision_points_back(sensor_msgs::PointCloud2 msg)
{
    coll_points_back = msg.width;
    // std::cout << "points cz received: " << cruz_points << std::endl;
}

/**
 * @brief Export trajectory info
 * 
 * @param msg 
 */
void ExtractTraj(std_msgs::Float64MultiArray msg)
{
    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);
    // bool _LINES_;
    // n.getParam("Param/LINES", _LINES_);
    // bool AP_right;
    // bool AP_left;
    // n.getParam("Param/AP_right", AP_right);
    // n.getParam("Param/AP_left", AP_left);
    bool _OVERTAKING_;
    int overtaking;
    n.getParam("Param/OVERTAKING", _OVERTAKING_);

    if (_OVERTAKING_ == true)
    {
        overtaking = 1;
    }
    else
    {
        overtaking = 0;
    }

    if (RECORDING == true)
    {
        if (car_number == 0)
        {
            n.getParam("car_number", car_number);
            sprintf(car_number_string, "%d", car_number);
        }

        char path_begin[100] = "/home/ana/catkin_ws/src/Multiple_AV/lartk5/planning/trajectory_planner/src/csv/traj_data_";
        char path_end[5] = ".csv";
        strcat(path_begin, car_number_string);
        strcat(path_begin, path_end);
        std::ofstream file_traj(path_begin, std::ios_base::app);

        // 0 -> index
        // 1 -> alpha
        // 2 -> max_angle
        // 3 -> speed

        // std::cout << ros::Time::now() << ",DLO: " << DLO << std::endl;

        file_traj << ros::Time::now() << "," << overtaking << "," << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "\n";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RetrieveInfo");
    ros::NodeHandle n("~");

    int RefreshRate = 30;
    n.getParam("RefreshRate", RefreshRate);

    ros::Rate loop_rate(RefreshRate);

    n.getParam("car_number", car_number);
    sprintf(car_number_string, "%d", car_number);

    min_dist_sub = n.subscribe("/analysis_data", 1, Extract_min_dist);

    clp_sub = n.subscribe("/line_close_point", 1, ExtractCLP2); //extract clp pos

    traj_sub = n.subscribe("/traj_data", 1, ExtractTraj); //Traj

    ap_sub = n.subscribe("/ap_data", 1, Extract_AP); // AP (needs ap_extract and clp_sub)

    ap_extract = n.subscribe("/Apoint", 1, get_AP); // Get AP data

    DS_sub = n.subscribe("/DS_data", 1, Extract_DS);

    Coll_points_front_sub = n.subscribe("/coll_marker", 1, Extract_collision_points_front);

    Coll_points_back_sub = n.subscribe("/coll_marker_back", 1, Extract_collision_points_back);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}