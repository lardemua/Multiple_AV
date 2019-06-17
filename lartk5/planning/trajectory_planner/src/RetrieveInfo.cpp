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

double pos_clp_x;
double pos_clp_y;
int lane;

// std::ofstream file_min_dist;
// std::ofstream file_speed;

void ExtractVel(gazebo_msgs::ModelStates models)
{

    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);

    if (RECORDING == true)
    {
        std::ofstream file_speed("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/speed.csv", std::ios_base::app);

        int pos_name = 0;

        int car_number = 0;
        n.getParam("car_number", car_number);
        char car_name[20] = "cirkit_unit03_h";
        char car_number_string[2];
        sprintf(car_number_string, "%d", car_number);
        strcat(car_name, car_number_string);

        double this_pos_x = 0;
        double this_pos_y = 0;
        double this_speed_new = 0;

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
        }

        // std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        file_speed << ros::Time::now() << "," << this_speed_new << "\n";
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RetrieveInfo");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(10);

    std::ofstream file_speed("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/speed.csv", std::ios_base::app);

    file_speed << "\n"
               << "new test"
               << "\n";

    std::ofstream file_min_dist("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/csv/analysis_data.csv", std::ios_base::app);

    file_min_dist << "\n"
                  << "new test"
                  << "\n";

    min_dist_sub = n.subscribe("/analysis_data", 1, Extract_min_dist);

    vel_sub = n.subscribe("/gazebo/model_states", 1, ExtractVel);

    clp_sub = n.subscribe("/line_close_point", 1, ExtractCLP2);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}