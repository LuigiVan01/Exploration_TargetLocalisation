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

1. Clone the repository:
   ```
   git clone https://github.com/LuigiVan01/metr4202_2024_team20.git
   ```

2. Navigate to the workspace directory:
   ```
   cd metr4202_2024_team20/search_algorithm
   ```

3. Build the workspace:
   ```
   colcon build
   ```

4. Source the workspace:
   ```
   source install/setup.bash
   ```

## Usage

To run the Autopilot package:

1. Launch Gazebo with your Turtlebot3 model (you can replace `turtlebot3_world.launch.py` with the appropriate launch file of the map you want to explore):
   ```
   ros2 launch metr4202_2024_team20 turtlebot3_world.launch.py
   ```

2. Launch Nav2 and SLAM Toolbox :
   ```
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
   ```

3. Run the Autopilot and Aruco Detection nodes together:
   ```
   ros2 launch metr4202_2024_team20 combined_launch.py
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

