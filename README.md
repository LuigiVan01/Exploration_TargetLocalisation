# Autopilot Package

This repository contains the Autopilot package, which is designed to send waypoints to a robot using SLAM Toolbox and Nav2 to explore an unknown map. The project is set up for use with Gazebo and ROS2 Humble, specifically using the Turtlebot3 robot.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributors](#contributors)

## Prerequisites

Before you begin, ensure you have Ubuntu 22.04 or compatible Linux distribution and the following software installed:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [Cartographer](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)  
- [Navigation2](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
- [Numpy](https://numpy.org/install/)


## Installation

To install this project, follow these steps:

1. If you haven't already, source the ros2 installation, and create a new directory for the workspace as below:
   ```
   source /opt/ros/humble/setup.bash
   mkdir -p ~/workspace_name/src
   ```
   
2. Clone the repository using the ubuntu terminal (if the terminal prompts you for credentials, sign into github within VS code etc, and use the built in terminal):
   ```
   git clone https://github.com/LuigiVan01/metr4202_2024_team20.git
   ```

2. Copy the "autopilot" and "aruco_detect" found in the repository, to the "src" folder in your workspace (your workspace should now have folders "src", "autopilot", and "aruco_detect"):

3. Build the workspace:
   ```
   cd ~/workspace_name/
   colcon build
   ```

4. Source the workspace (and the gazebo setup if you haven't already):
   ```
   source ~/workspace_name/install/setup.bash
   #code below is for sourcing the gazebo setup and is optional if you've already done it
   source /usr/share/gazebo/setup.sh
   ```

5.

## Running Autopilot and Detection in Gazebo Simulations

To run the Autopilot package:

1. Launch Gazebo with your Turtlebot3 model (you can replace `turtlebot3_world.launch.py` with the appropriate launch file of the map you want to explore, this is located by default in: /metr4202_2024_team20/src):
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Launch Nav2 and SLAM Toolbox :
   ```
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
   ```

3. Launch the Autopilot node:
   ```
   ros2 launch metr4202_2024_team20 autopilot_launch.py
   ```

4. Launch the Aruco Detection node:
   ```
   ros2 launch metr4202_2024_team20 aruco_launch.py
   ```
## Running Autopilot and Detection within Physical Environment

To run the Autopilot package:

1. Launch Gazebo with your Turtlebot3 model (you can replace `turtlebot3_world.launch.py` with the appropriate launch file of the map you want to explore):
   ```
   ros2 launch metr4202_2024_team20 turtlebot3_world.launch.py
   ```

2. Launch Nav2 and SLAM Toolbox :
   ```
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
   ```

3. Launch the Autopilot node:
   ```
   ros2 launch metr4202_2024_team20 autopilot_launch.py
   ```

4. Launch the Aruco Detection node:
   ```
   ros2 launch metr4202_2024_team20 aruco_launch.py
   ```

## Project Structure

```
metr4202_2024_team20/
├──autopilot/
│   └── autopilot/
│       └── autopilot.py
│       ├── package.xml
│       └── setup.py   
│── README.md
└── ignore
```

## Contributors
  Team 20:

 - Luigi Vanacore         48543518
 - ZhuoXin Shi            48969761
 - Francis Weber          46992288
 - Ricardo Westerman      45829453
 - Rui Xia                43507917

