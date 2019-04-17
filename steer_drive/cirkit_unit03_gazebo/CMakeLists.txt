# CMakeLists.txt
#
# Copyright (c) 2015 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(cirkit_unit03_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roslaunch
	roscpp
        rospy
        std_msgs
	lidar_segmentation
	colormap
        pcl_ros
        velodyne_pointcloud
        tf_conversions)
find_package(cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES free_space_detection
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime colormap pcl_ros velodyne_pointcloud
  DEPENDS Eigen
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        include
)
      
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ackermann_vehicle_gazebo.cpp)
add_executable(tf_generator4 src/tf_generator4.cpp)
add_executable(tf_generator5 src/laser_link_generator.cpp)
add_executable(tf_generator6 src/line_link_generator.cpp)
add_executable(cirkit_unit03_gazebo_line_node src/ackermann_vehicle_gazebo_line.cpp)

add_dependencies(tf_generator4 trajectory_planner_generate_messages_cpp)
add_dependencies(tf_generator5 trajectory_planner_generate_messages_cpp)
add_dependencies(tf_generator6 trajectory_planner_generate_messages_cpp)
add_dependencies(cirkit_unit03_gazebo_line_node trajectory_planner_generate_messages_cpp)

      
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
${Eigen_LIBRARIES}
)
target_link_libraries(tf_generator4 ${catkin_LIBRARIES})
target_link_libraries(tf_generator5 ${catkin_LIBRARIES})
target_link_libraries(tf_generator6 ${catkin_LIBRARIES})
target_link_libraries(cirkit_unit03_gazebo_line_node ${catkin_LIBRARIES})

#roslaunch_add_file_check(test/cirkit_unit03_clearpath_playpen_launch.test.xml)
#roslaunch_add_file_check(test/cirkit_unit03_world_launch.test.xml)
#roslaunch_add_file_check(test/description.gazebo_launch.test.xml)
