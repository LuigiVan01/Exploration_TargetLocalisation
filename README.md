# Autopilot Package

This repository contains the Autopilot package, which is designed to send waypoints to a robot using SLAM Toolbox and Nav2 to explore an unknown map. The project is set up for use with Gazebo and ROS2 Humble, specifically using the Turtlebot3 robot.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Prerequisites

Before you begin, ensure you have met the following requirements:
* Ubuntu 22.04 or compatible Linux distribution
* ROS2 Humble installed
* Gazebo installed
* SLAM Toolbox and Nav2 packages installed
* Turtlebot3 packages installed

Before you start, ensure that you have the following software installed:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [slam_toolbox](https://navigation.ros.org/setup_guides/slam_config.html)
- [nav2](https://navigation.ros.org/getting_started/index.html)

## Installation

To install this project, follow these steps:

1. Clone the repository:
   ```
   git clone https://github.com/LuigiVan01/metr4202_2024_20.git
   ```

2. Navigate to the workspace directory:
   ```
   cd metr4202_2024_20/search_algo
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

1. Launch Gazebo with your Turtlebot3 model (replace `your_robot.launch.py` with the appropriate launch file):
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Launch SLAM Toolbox:
   ```
   ros2 launch slam_toolbox online_async_launch.py
   ```

3. Launch Nav2:
   ```
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
   ```

4. Run the Autopilot node:
   ```
   ros2 run autopilot autopilot
   ```

## Project Structure

```
metr4202_2024_20/
├── search_algo/
│   ├── src/
│   │   └── autopilot/
│   │       ├── autopilot/
│   │       │   └── autopilot.py
│   │       ├── package.xml
│   │       └── setup.py
│   ├── build/
│   ├── install/
│   └── log/
└── README.md
```

## Contributors
  Team 20:

 - Luigi Vanacore         48543518
 - ZhuoXin Shi            48969761
 - Francis Weber          46992288
 - Ricardo Westerman      45829453
 - Rui Xia                43507917


## License

[Include your license information here]
