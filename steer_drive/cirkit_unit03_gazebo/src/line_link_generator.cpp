/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/**
 * @file line_link_generator.cpp
 * @brief Transform frame generator
 * @author Ricardo Silva
 * @version v0
 * @date 2018-06-06
 */

/**
 * @file line_link_generator.cpp
 * @brief Transform frame generator
 * @author Manuel Ferreira
 * @version v1
 * @date 2019-07-03
 */

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
tf::TransformBroadcaster* p_broadcaster6;

/**
 * @brief TF generator between "/world" and "/line_link", line frame
 * @param int
 * @param char**
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_generator6_node");
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster6;
  p_broadcaster6 = &broadcaster6;
  ros::Rate r(50);

  int car_number = 0;
  n.getParam("car_number", car_number);
  char link_name[20] = "/line_link_";
  char car_number_string[2];
  sprintf(car_number_string, "%d", car_number);
  strcat(link_name, car_number_string);

  tf::Transform transform1(tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), tf::Vector3(-1.82, -5.30185, 0));
  // tf::Transform transform1(tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), tf::Vector3(-1.82, -6.1, 0));
  while (n.ok())
  {
    p_broadcaster6->sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/world", link_name));
    r.sleep();
    ros::spinOnce();
  }
}