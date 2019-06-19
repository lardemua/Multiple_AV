#include <fstream>
#include <stdlib.h> /* atof */
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "gazebo_msgs/ModelStates.h"
#include <trajectory_planner/coordinates.h>
//-----------------------------------

ros::Subscriber min_dist_sub;
ros::Subscriber vel_sub;
ros::Subscriber clp_sub;
ros::Subscriber traj_sub;
ros::Subscriber ap_sub;
ros::Subscriber ap_extract;
ros::Subscriber DS_sub;

double pos_clp_x;
double pos_clp_y;
int lane;

double ap_x;
double ap_y;
double dist_ap;

// std::ofstream file_min_dist;
// std::ofstream file_speed;

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
        std::ofstream file_ap("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/ap.csv", std::ios_base::app);

        // std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        // 0 -> speed

        file_ap << ros::Time::now() << "," << ap_lane << "," << ap_x << "," << ap_y << "," << dist_ap << "," << msg.data[0] << "\n";
    }
}

void Extract_DS(std_msgs::Float64MultiArray msg)
{

    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);

    if (RECORDING == true)
    {
        std::ofstream file_DS("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/DS.csv", std::ios_base::app);

        // std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        // 0 -> count_points_detected_front,
        // 1 -> count_points_detected_back,
        // 2 -> limit_left,
        // 3 -> limit_right,
        // 4 -> limit_left_back,
        // 5 -> limit_right_back,
        // 6 -> detect_front,
        // 7 -> detect_back

        file_DS << ros::Time::now() << "," << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "," << msg.data[4] << "," << msg.data[5] << "," << msg.data[6] << "," << msg.data[7] << "\n";
    }
}

void Extract_min_dist(std_msgs::Float64MultiArray msg)
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
    // bool _OVERTAKING_;
    // n.getParam("Param/OVERTAKING", _OVERTAKING_);

    if (RECORDING == true)
    {
        std::ofstream file_min_dist("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/analysis_data.csv", std::ios_base::app);

        // double DLO = msg.data[1];

        // 0 -> time::now()
        // 1 -> DLO
        // 2 -> alpha
        // 3 -> index
        // 4 -> speed
        // 5 -> max_angle

        // std::cout << ros::Time::now() << ",DLO: " << DLO << std::endl;

        file_min_dist << ros::Time::now() << "," << msg.data[0] << "," << msg.data[1] << "," << msg.data[2] << "," << msg.data[3] << "," << msg.data[4] << "," << lane << "\n";
    }
}

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

void get_AP(trajectory_planner::coordinates msg)
{
    ap_x = msg.x;
    ap_y = msg.y;
    dist_ap = sqrt(pow(ap_x, 2) + pow(ap_y, 2));
}

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
        std::ofstream file_traj("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/traj_data.csv", std::ios_base::app);

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

    // std::ofstream file_ap("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/ap.csv", std::ios_base::app);

    // file_ap << "\n"
    //         << "new test"
    //         << "\n";

    // std::ofstream file_min_dist("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/analysis_data.csv", std::ios_base::app);

    // file_min_dist << "\n"
    //               << "new test"
    //               << "\n";

    // std::ofstream file_traj("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/traj_data.csv", std::ios_base::app);

    // file_traj << "\n"
    //           << "new test"
    //           << "\n";

    // min_dist_sub = n.subscribe("/analysis_data", 1, Extract_min_dist);

    // vel_sub = n.subscribe("/gazebo/model_states", 1, ExtractVel);

    // clp_sub = n.subscribe("/line_close_point", 1, ExtractCLP2);

    // traj_sub = n.subscribe("/traj_data", 1, ExtractTraj);

    // ap_sub = n.subscribe("/ap_data", 1, Extract_AP);

    // ap_extract = n.subscribe("/Apoint", 1, get_AP);

    DS_sub = n.subscribe("/DS_data", 1, Extract_DS);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}