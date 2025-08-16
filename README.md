# Exploration and Target Localisation

This repository contains the `autopilot_package`, `aruco_package` and `autopilot_physical_package`. The `autopilot_package` is designed to send waypoints to a robot using SLAM Toolbox and Nav2 to explore an unknown map. The `aruco_package` allows to detect and localise aruco targets in the map. The `autopilot_physical_package` is the version that contains the detection node which is  advisible to be run on the robot. The project is set up for use with Gazebo and ROS2 Humble, specifically using the Turtlebot3 robot. `brief_report.pdf` is a short report about how the codes work.

![Demo](demo.gif)

Different waypoints (shown in purple in the demo) are evaluated based on their proximity to obstacles and whether they lie on the exploration frontier. If, during exploration, the algorithm detects a marker (shown in red), it immediately generates a waypoint directed toward the detection. This allows the robot to collect additional measurements of the marker’s position, leading to a more accurate final estimation.

Different waypoints are evaluated based on their closeness to obstacles and their being in the frontier. The waypoints are coloured in purple in the gif. If during the exploration the algorithm detects a marker (coloured in red in the gif), it sends immediately a waypoints toward the detection of the detected marker so that the robot can get more measurements of the position of the marker and so have a better final estimation of the position.

## Getting Started

There are two recommended ways to set up this project: using Docker for a streamlined experience, or a manual installation.

### Option 1: Using Docker (Recommended)

The included Docker setup is the most straightforward method to get the project running, as it handles all dependencies within a container. It is highly recommended to use the [Dev Containers extension in VSCode](https://code.visualstudio.com/docs/devcontainers/containers) for a seamless workflow.

**Prerequisites for Docker Setup:**

*   [Docker](https://docs.docker.com/get-docker/)
*   [Visual Studio Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

**Usage with Docker:**

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/LuigiVan01/Turtlebot3Exploration_TargetLocalisation.git
    ```

2.  **Open in VSCode and launch the Dev Container:**
    Open the cloned repository folder in VSCode. The Dev Containers extension will detect the `.devcontainer` configuration and prompt you to "Reopen in Container". Click this to build and launch the container.

3.  **Build the ROS2 packages:**
    Once inside the container, a terminal will be available. You will already be in the `turtlebot3_ws` directory. Build the packages and source the workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```

4.  **Allow GUI from Docker:**
    In a terminal on your **local machine** (not in the container), run the following command:
    ```bash
    xhost +local:docker
    ```

5.  **Launch the Gazebo world:**
    In a terminal within the VSCode container, launch one of the available Gazebo worlds. For example:
    ```bash
    ros2 launch autopilot_package metr4202_world.launch.py
    ```
    *Note: You can find other available worlds in the `autopilot_package/launch` directory.*

6.  **Start Navigation and SLAM:**
    In a **new terminal** inside the container, launch Navigation2 and the SLAM Toolbox:
    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
    ```

7.  **Launch the Aruco Detection Node:**
    In another terminal inside the container, start the Aruco marker detection:
    ```bash
    ros2 launch aruco_package aruco.launch.py
    ```

8.  **Launch the Autopilot Node:**
    Finally, in a new terminal, launch the autopilot node to begin the exploration process:
    ```bash
    ros2 launch autopilot_package autopilot.launch.py
    ```

### Option 2: Manual Installation

If you prefer a manual setup, you will need to install all the necessary dependencies on your system.

**Prerequisites for Manual Installation:**

*   Ubuntu 22.04 or a compatible Linux distribution.
*   [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
*   [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
*   [Cartographer](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
*   [Navigation2](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
*   [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
*   [Numpy](https://numpy.org/install/)

**Usage with Manual Installation:**

1.  **Clone the repository into your ROS2 workspace:**
    Create a ROS2 workspace if you don't have one, and clone the repository into the `src` folder.
    ```bash
    mkdir -p turtlebot3_ws/src
    cd turtlebot3_ws/src
    git clone https://github.com/LuigiVan01/Turtlebot3Exploration_TargetLocalisation.git
    ```

2.  **Build the packages:**
    Navigate to the root of your workspace, build the packages, and source the setup file.
    ```bash
    cd .. 
    colcon build
    source install/setup.bash
    ```

3. Set the domain ID and Gazebo model paths (don't forget to change "workspace_name" to your workspace's name):
   ```
   export ROS_DOMAIN_ID=30 #TURTLEBOT3
   export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/workspace_name/src/turtlebot3_simulations/turtlebot3_gazebo/models
   export TURTLEBOT3_MODEL=waffle_pi
   ```
4. Add all the source and export commands to your .bashrc so they do not need to be called in future:
    ```
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    echo 'source ~/workspace_name/install/setup.bash' >> ~/.bashrc
    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/workspace_name/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc   
    ```

5.  Follow steps 5 through 8 from the Docker usage guide, running each `ros2 launch` command in a new terminal.

## Running Autopilot and Detection within a Physical Environment

The following steps assume that you are using a computer on which you have followed the installation steps above.

1. ssh into your Turtlebot and enter your credentials when prompted:
   ```
   ssh ubuntu@Turtlebot3_IP_address
   ```
   
2. As in the installation instructions, create a new workspace and source the humble setup (again this workspace will need to have the "Turtlebot3 Simulations" packages installed in the "src" folder):
   ```
   source /opt/ros/humble/setup.bash
   mkdir -p ~/workspace_name/src
   ```
   
3. Within your Turtlebot's "src" directory, clone the repository:
    ```
   cd ~/workspace_name/src
   git clone https://github.com/LuigiVan01/metr4202_2024_team20.git
   ```
    
4. Build the workspace and source the setup
   ```
   cd ~/workspace_name/
   colcon build
   source ~/workspace_name/install/setup.bash
   ```
   
5. Your workspace on the Turtlebot3 should now look like this:
   ```
   workspace_name
   └── src
       ├── autopilot_package
       ├── aruco_package
       ├── autopilot_physical_package
       ├── DynamixelSDK
       ├── turtlebot3
       ├── turtlebot3_msgs
       └── turtlebot3_simulations
    ```
   
6. In the Turtlebot3 ssh terminal run the robot launch file:
   ```
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
7. In a new terminal tab or window (non-ssh), set the domain ID and start navigation2 and slam.
   ```
   export ROS_DOMAIN_ID=<ID>
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=True
   ```
8. In a new terminal tab or window (non-ssh), set the domain ID and run the autopilot package.
   ```
   export ROS_DOMAIN_ID=<ID>
   ros2 launch autopilot_package autopilot.launch.py
   ```
9. In a new Turtlebot3 ssh terminal launch:
   ```
   ros2 run v4l2_camera v4l2_camera_node
   ```
10. In a new ssh terminal tab or window, launch the aruco detection node.
   ```
   ros2 launch autopilot_physical_package aruco_robot.launch.py
   ```

# Custom Maps
We have created a series of custom maps for use in testing our code. They are located in the "autopilot_package" in the "worlds" folder, and can be run with the following commmand (launch file names can be found in the "launch" folder in the "autopilot_package"):

```
ros2 launch autopilot_package <launch-file-name>
```
## This is a preview of some of the maps:

### labmap
![image](https://github.com/user-attachments/assets/96706ead-38aa-41f7-a2ed-0166d7934a03)

### labmap3
![image](https://github.com/user-attachments/assets/b5065c88-3e45-4ef4-a327-7ffc2e1e5638)

### Project 2

![image](https://github.com/user-attachments/assets/3b95c2cd-ff7f-4cb1-9621-fefabce8e64f)

### Narrows

![image](https://github.com/user-attachments/assets/94c842b1-d087-40ad-a1eb-40d52d192721)


# Contributors

 - Luigi Vanacore (LuigiVan01)      
 - ZhuoXin Shi (zhuobykt)           
 - Francis Weber (frankotothe)      
 - Ricardo Westerman (ricowest)     
 - Rui Xia  (RayAutisticSoloXia)    

