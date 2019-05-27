#include <fstream>
#include <stdlib.h> /* atof */
#include <string>
#include <iostream>

int main()
{

    double waypoints[1000][2];

    std::ifstream sourcefile("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/XeY.csv");

    std::ofstream outputfile("/home/manuel/catkin_ws/src/lartk5/planning/trajectory_planner/src/XeY1000.csv");

    if (!sourcefile.is_open())
    {
        std::cout << "ERROR: File Open" << '\n';
    }

    std::string posX;
    std::string posY;
    double pos_x, pos_y;
    int count = 0;
    int countline = 0;

    while (sourcefile.good())
    {

        getline(sourcefile, posX, ',');
        getline(sourcefile, posY, '\n');

        pos_x = atof(posX.c_str());
        pos_y = atof(posY.c_str());

        if (count % 1000 == 0)
        {
            outputfile << pos_x << "," << pos_y << "\n";
        }


        // ROS_INFO("READ FILE");

        count++;
    }
    // ROS_INFO("READ FILE");

    sourcefile.close();
    return 0;
}