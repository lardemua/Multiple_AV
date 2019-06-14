#include <fstream>
#include <stdlib.h> /* atof */
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "gazebo_msgs/ModelStates.h"

//-----------------------------------

ros::Subscriber min_dist_sub;
ros::Subscriber vel_sub;

void ExtractVel(gazebo_msgs::ModelStates models)
{

    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);

    if (RECORDING == true)
    {
        std::ofstream file_speed("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/speed.csv");

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

        std::cout << ros::Time::now() << ",speed: " << this_speed_new << std::endl;

        // file_speed << ros::Time::now() << "," << this_speed_new << "\n";
    }
}

void Extract_min_dist(std_msgs::Float64MultiArray msg)
{
    ros::NodeHandle n;
    bool RECORDING;
    n.getParam("Param/RECORDING", RECORDING);

    if (RECORDING == true)
    {
        std::ofstream file_min_dist("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/min_dist.csv");

        double DLO = msg.data[1];

        std::cout << ros::Time::now() << ",DLO: " << DLO << std::endl;

        // file_min_dist << ros::Time::now() << "," << DLO << "\n";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RetrieveInfo");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(10);

    min_dist_sub = n.subscribe("/analysis_data", 1, Extract_min_dist);

    vel_sub = n.subscribe("/gazebo/model_states", 1, ExtractVel);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}