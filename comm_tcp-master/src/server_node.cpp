/*************************************************************************
* Author: Abhinav Jain
* Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
* Date: 28/06/2016
*
* This file contains source code to the server node of the ROS package
* comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
* as part of my project during an internship from May 2016 - July 2016.
*
* (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
***************************************************************************/
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include "std_msgs/String.h"

using namespace std;

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "server_node");
  ros::NodeHandle nh;
  ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages/", 1000);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/steer_drive_controller/cmd_vel", 1);
  // Testing package is working fine
  int sockfd, newsockfd, portno;           // Socket file descriptors and port number
  socklen_t clilen;                        // object clilen of type socklen_t
  char buffer[256];                        // buffer array of size 256
  struct sockaddr_in serv_addr, cli_addr;  /// two objects to store client and server address
  std_msgs::String message;
  std::stringstream ss;
  int n;
  ros::Duration d(0.01);  // 100Hz
  if (argc < 2)
  {
    fprintf(stderr, "ERROR, no port provided\n");
    exit(1);
  }
  portno = atoi(argv[1]);
  cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");
  listen(sockfd, 5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
  if (newsockfd < 0)
    error("ERROR on accept");

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ss.str(std::string());  // Clear contents of string stream
    bzero(buffer, 256);
    n = read(newsockfd, buffer, 255);
    if (n < 0)
      error("ERROR reading from socket");
    // printf("Here is the message: %s\n",buffer);
    ss << buffer;
    message.data = ss.str();
    ROS_INFO("%s", message.data.c_str());
    server_pub.publish(message);

    geometry_msgs::Twist twist_msg;

    char init = message.data.at(0);

    if (int(init) == 118)
    {
      char end = message.data.at(10);
      double speed = (message.data.at(1) - 48) * 1 + (message.data.at(3) - 48) * 0.1 + (message.data.at(4) - 48) * 0.01;
      double steer;

      if (int(end) == 77)
      {
        steer = (message.data.at(6) - 48) * 1 + (message.data.at(8) - 48) * 0.1 + (message.data.at(9) - 48) * 0.01;
      }
      if (int(end) == 109)
      {
        steer = -((message.data.at(6) - 48) * 1 + (message.data.at(8) - 48) * 0.1 + (message.data.at(9) - 48) * 0.01);
      }

      twist_msg.angular.z = steer;
      twist_msg.linear.x = speed;

      twist_pub.publish(twist_msg);

      n = write(newsockfd, "I got your message", 18);
      if (n < 0)
        error("ERROR writing to socket");
    }

    if (int(init) == 113)
    {
      ROS_INFO("Stopping connection..");
      ROS_INFO("Exiting..");

      n = write(newsockfd, "Server stopped", 14);
      if (n < 0)
        error("ERROR writing to socket");

      close(newsockfd);
      close(sockfd);

      return 0;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
