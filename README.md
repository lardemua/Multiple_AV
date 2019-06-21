# Multiple_AV

Trajectory planner of the ATLASCAR2

# About

Mechanical Engineering Master's Thesis

* Author: **Manuel Teles Ferreira**

Laboratory of Automation and Robotics (LAR)

Department of Mechanical Engineering

University of Aveiro

# Requirements

* [ROS (Melodic)](http://wiki.ros.org/melodic/Installation/Ubuntu)

* Gazebo:

```
sudo apt install gazebo9*
```

* Gazebo (ROS):

```
sudo apt install ros-melodic-gazebo-*
```

* [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

* Git:

```
sudo apt install git
```

# Setup the packages

Go to the catkin_ws folder and run:

```
git clone https://github.com/lardemua/Multiple_AV.git
```

Now you should have a folder named Multiple_AV.

Copy everything inside to the src folder (catkin_ws/src)

# Rosdep

So that you install all the packages required, in catkin_ws run:

```
rosdep install --from-paths src --ignore-src -r -y
```

This will search the src folder for all the dependencies and install everything required.

# Compiling

First you need to compile required packages (run with -j8 or similar): 

```
catkin_make mtt
```

```
catkin_make atlasmv_base
```

Now just run:

```
catkin_make
```

(Depending on the number of threads used, there may occur errors for missing packages. If this is the case just run catkin_make again).

# Map placement

Just like mentioned in the Readme.txt in the Monaco folder you'll need to follow 2 steps in order for the map to be accecible by gazebo:

In catkin_ws/src run the following code that will place the map in the correct gazebo folder:

```
sudo cp -r Monaco /opt/ros/melodic/share/gazebo_ros/launch
```

Next in /opt/ros/melodic/share/gazebo_ros/launch run the following code so that Gazebo can access the files:

```
sudo chmod -R ugo+rw Monaco
```

# Running the program

To launch the program run:

```
roslaunch cirkit_unit03_gazebo ackermann_vehicle_simulator.launch
```

# Editing details

In the ackermann_vehicle_simulator.launch file you can specify the parameters values depending on the simulation desired.

# Information:

[My Blog](https://planeamentotrajetorias.wordpress.com/)

[My GitHub](https://github.com/ManuelTFerreira)


