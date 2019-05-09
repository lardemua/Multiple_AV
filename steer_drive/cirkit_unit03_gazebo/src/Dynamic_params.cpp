#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <cirkit_unit03_gazebo/MyParamsConfig.h>

void callback(cirkit_unit03_gazebo::MyParamsConfig &config, uint32_t level)
{
    // ROS_INFO("New values: [%d]", config.int_param, config.SPEED_SAFFETY.);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Dynamic_params");

    dynamic_reconfigure::Server<cirkit_unit03_gazebo::MyParamsConfig> server;
    dynamic_reconfigure::Server<cirkit_unit03_gazebo::MyParamsConfig>::CallbackType f;

    f=boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    
    return 0;
}