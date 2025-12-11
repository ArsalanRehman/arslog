# Setup & Usage Guide

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**

---

## Install Nav2 Packages

Use the following command:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```

Add this to your `.bashrc`:

```bash
export TURTLEBOT3_MODEL=waffle
```

---

## Launch TurtleBot3 Simulation

To open the TurtleBot3 simulation environment:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## Teleoperation

To open the teleop node (move the robot):

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Start Mapping

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

---

## Save Map

After mapping, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

## Issues with nav2

- ros2 humble is not really stable yet so there are some known issues with nav2 e.g map not getting loaded on time or not loaded at all.

- To fix this we need to install the following packages:

```bash
sudo apt update && sudo apt install ros-humble-rmw-cyclonedds-cpp
```

After that add this line to the .bashrc file:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

After that we need to make change inside waffle.yaml file inside `/opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml`

change the line `robot_model_type: differential` to `robot_model_type: nav2_amcl::DifferentialMotionModel`

Reboot the PC after making that changes.

## Navigation Stack

Now lets start the navigation stack:

Lets Open the gazebo environment:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Use the following command to start the navigation (you need to locallize the robot first) :

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/epik-software/maps/my_map.yaml
```
# Understanding the Nav2 Stack

- **ros-humble-navigation2** : This package provides the navigation stack for ROS 2.
- **ros-humble-nav2-bringup** : This package provides the launch files for the navigation stack.
- **ros-humble-turtlebot3** : This package provides the TurtleBot3 packages or simulation or testing, you'll need this package to gain access to all the TurtleBot 3 components. . 

### Global Planner vs Local Planner and Cost Maps 
 - **Global Planner**: This component computes a path to a specified goal using the entire map and a cost map that assigns costs to each pixel based on proximity to obstacles. It aims to find the lowest cost path while avoiding high-cost areas.

- **Local Planner (Controller)**: The local planner receives the path from the global planner and controls the robot's movements in real time. It updates more frequently than the global planner in order to make responsive adjustments as the robot navigates its environment.

- **Analogy with GPS**: The global planner is likened to a GPS in a car that computes the route, while the local planner acts like the driver, making real-time decisions during navigation.

- **Cost Maps**: Both planners use cost maps that influence navigation decisions, impacting how paths are determined and followed.
- **Update Frequencies**: The global planner updates the path approximately once per second, while the local planner operates at a much higher frequency for better responsiveness.

- **Parameters**: In order to manipulate the behavior of nav2 you can access the parameters using rqt gui. plugins -> Configuration -> Dynamic Reconfigure. After that expand global_costmap and select global_costmap.

---

# Building Your Own World for Navigation in Gazebo

*  

