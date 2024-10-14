import numpy as np
import rclpy
import math
import time
from random import randrange
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import subprocess
from geometry_msgs.msg import Twist


class Autopilot(Node):

    def __init__(self):

        super().__init__('autopilot')
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_camera_parameters("/home/szx/ros2_ws/calibration_file.yaml")
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        
        self.queue_size = 10

        # 初始化cv_bridge
        self.bridge = CvBridge()

        # 初始化aruco字典
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 订阅图像话题
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            self.queue_size
        )

        """Initialize autopilot parameters:

        potential_pos (OccupancyGrid): Occupancy grid index associated with next potential position

        occupancy_grid (OccupancyGrid): Current occupancy grid information received from '/map'. References callback function 'next_waypoint'
        
        """
        # Allow callback functions to be called in parallel
        #self.parallel_callback_group = ReentrantCallbackGroup()

        # Initilizing the probablity at which we consider there to be an obstacle
        self.obstacle_probability = 85

        # Specifies how many incoming messages should be buffered
        self.queue_size = 10

        # Flag variable that indicates that the exploration is just started
        self.start=True

        # Initialize the number of waypoints published
        self.waypoint_counter = 0

        # Initialize the number of waypoints published with the new strategy
        self.new_strategy_counter = 0

        # Array of indexes already checked
        self.occupancy_data_np_checked = []

        # Initialize the current grid
        self.current_grid = OccupancyGrid()

        # Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0

        # Initializing potential_coordinate for debugging
        self.potential_coordinate = PointStamped()
        self.potential_coordinate.header.frame_id = 'map'
        self.potential_coordinate.point.x = 0.0
        self.potential_coordinate.point.y = 0.0

        #Initializing current position variable
        self.current_position = PointStamped()
        self.current_position.header.frame_id = 'map'

        #Initializing number of iterations before the strategy is changed
        self.strategy_counter = 10

        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.readiness_check,
            self.queue_size,
            #callback_group=self.parallel_callback_group
        )

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.store_grid,
            self.queue_size
        )

        #Subscribe to /pose to determine position of Turtlebot
        self.position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.current_position_callback,
            self.queue_size,
            #callback_group=self.parallel_callback_group
        )

        
        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            self.queue_size
        )

        #Publisher for publishing potential waypoints to for bug fixing
        self.potential_publisher = self.create_publisher(
            PointStamped,
            'potential_point',
            self.queue_size
        )

        #Publisher for publishing current coordinate for bug fixing
        self.current_publisher = self.create_publisher(
            PointStamped,
            'current_point',
            self.queue_size
        )

        # Publisher for publishing already checked points to for bug fixing
        self.checked_points_publisher = self.create_publisher(
            PointCloud2,
            'checked_points',
            self.queue_size
        )



    
    
    def store_grid(self,grid:OccupancyGrid):
        """ 
            Callback function of /map topic.
            Everytime it receives the OccupacyGrid message it stores it.
            At the start it launches the next_point() method since no message is received from the behavior_tree_log.
        """
        self.current_grid=grid

        #Initiates looking for new waypoint if exploration has just started.
        #This is because readiness_check will not do this when exploration has just started
        if self.start:
            self.next_waypoint()
            self.start = False
            
            

    def next_waypoint(self):
        """
        Function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        """

        self.width = self.current_grid.info.width
        isthisagoodwaypoint = False

        not_in_range_count=0
        max_iterations = 50000  # 限制迭代次数，避免无限循环

        min_distance = 1
        max_distance = 3

        still_looking = False
        occupancy_data_np = np.array(self.current_grid.data)

        if self.strategy_counter > 0:
            iterations = 0
            while not isthisagoodwaypoint and iterations < max_iterations:
                iterations += 1
                self.occupancy_data_np_checked = []
                # Added this so that the terminal isn't filled with messages so it's easier to read
                if not still_looking:
                    self.get_logger().info('Searching for good point...')
                    still_looking = True

                # Taking a random cell and the corresponding cost value
                random_index = randrange(occupancy_data_np.size)
                self.potential_pos = occupancy_data_np[random_index]

                # Check that the cell has not been checked before or is unknown
                if random_index in self.occupancy_data_np_checked or self.potential_pos == -1:
                    continue

                # Compute correspondent row and column of the potential cell 
                [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(random_index)
                
                # Publish current position and potential position for visualization
                self.potential_publisher.publish(self.potential_coordinate)
                #self.get_logger().info('Is the point published?')
                #time.sleep(0.5)
                self.current_publisher.publish(self.current_position)

                # Check that the cell is not an obstacle
                if self.potential_pos>= self.obstacle_probability:
                    
                    self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, random_index)
                    continue
                
                # Check that the point is on the frontier
                elif not self.frontier_check(occupancy_data_np, random_index):

                    #self.get_logger().info('Point was not on frontier')

                    self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, random_index)
                    continue
    
                else:
                    self.get_logger().info('Found Good Point')
                    self.get_logger().info('Checking Point Distance')

                    distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.point.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.point.y)**2
                    )


                    if min_distance < distance2new < max_distance or self.start:
                        self.new_waypoint.pose.position.x = self.potential_coordinate.point.x
                        self.new_waypoint.pose.position.y = self.potential_coordinate.point.y
                        self.get_logger().info('Point Distance:' + str(distance2new))
                
                        isthisagoodwaypoint = True

                        #Put a box around the published point in the array of the already checked points
                        self.box_checked(random_index) 
                        self.strategy_counter -= 1
                        self.get_logger().info("Remaining points before new strategy:" + str(self.strategy_counter))

                    else:
                        self.get_logger().info('Point not in range, Finding New...')
                        isthisagoodwaypoint = False
                        still_looking = False

                        # If the point is not in range for 30 iterations, adopt a new strategy
                        not_in_range_count += 1
                        if not_in_range_count > 50:
                            self.get_logger().info('Could not find point in range, adopting new strategy...')
                            time.sleep(3)
                            self.new_strategy()
                            isthisagoodwaypoint = True
                            
            
                            
            if iterations >= max_iterations:
                self.get_logger().info("No more good points, SLAM completed.")
                self.on_slam_complete()
                return
                            
                    
                            
        #New strategy
        else:
            self.strategy_counter = 5
            self.new_strategy()
           
        #Publish the new waypoint
        self.get_logger().info('Publishing waypoint...')
        self.waypoint_publisher.publish(self.new_waypoint)
        self.waypoint_counter += 1

        if self.waypoint_counter % 10 == 0:
            # Create a list to store all points
            points = []

            # Convert all indices to points (only x and y coordinates)
            for index in self.occupancy_data_np_checked:
                x, y = self.cell_coordinates(index)
                points.append([x, y, 0.0])  # z=0 for 2D map

            # Convert points to numpy array
            points_np = np.array(points, dtype=np.float32)

            # Create the header with just the frame_id
            header = Header()
            header.frame_id = 'map'

            # Create and publish the point cloud
            pc2 = point_cloud2.create_cloud_xyz32(header, points_np)
            self.checked_points_publisher.publish(pc2)


    def new_strategy(self):
        """Processes the occupancy grid and creates a sorted list of cells based on the number of uncertain cells around them."""

        self.get_logger().info('New Strategy: Processing occupancy grid ...')
        occupancy_data_np = np.array(self.current_grid.data)
        width = self.current_grid.info.width
        height = self.current_grid.info.height
        counts_list = []
        self.occupancy_data_np_checked = []
        # Iterate over all cells in the occupancy grid
        for index in range(len(occupancy_data_np)) :
            
            if index in self.occupancy_data_np_checked or occupancy_data_np[index] == -1:
                continue


            if occupancy_data_np[index] >= self.obstacle_probability:
                self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, index)
                continue

            [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(index)

            distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.point.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.point.y)**2
                    ) 

            # Check if the distance is greater than 9 meters and counter is not a multiple of four
            if distance2new > 9 and self.new_strategy_counter % 4 != 0:
                continue

            else: 
                # Count the number of uncertain cells around the current cell and append to the list
                uncertain_count = self.count_uncertain_cells_around(index, occupancy_data_np, width, height)
                counts_list.append((index, uncertain_count))
                if uncertain_count==0:
                    self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, index)

        # Sort the list by uncertain_count in descending order
        sorted_counts = sorted(counts_list, key=lambda x: x[1], reverse=True)
        

        [self.new_waypoint.pose.position.x,self.new_waypoint.pose.position.y]= self.cell_coordinates(sorted_counts[0][0])
        self.box_checked(sorted_counts[0][0])
        [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(sorted_counts[0][0])

        distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.point.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.point.y)**2
                    )
        
        self.get_logger().info('New Strategy: Point Distance:' + str(distance2new))
        self.new_strategy_counter += 1
        self.potential_publisher.publish(self.potential_coordinate)
        


    def count_uncertain_cells_around(self, index, occupancy_data_np, width, height):
        """Counts the number of uncertain cells (-1) around a given cell within a defined box size."""
        uncertain_count = 0
        box_size = 5  # Define the size of the box around each cell

        # Loop over the box around the cell
        for x in range(-box_size, box_size + 1):
            for y in range(-box_size, box_size + 1):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[index + slider] == -1:
                        uncertain_count += 1
                #the index of a point next to the index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
        return uncertain_count
           

    def frontier_check(self, occupancy_data_np, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes  = 0

        #Inspects the nature of points in a grid around the selected point 
        for x in range(-4,3):
            for y in range(-4,3):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[random_index + slider] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data_np[random_index + slider] > self.obstacle_probability:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
         
        if uncertain_indexes > 20:
            return True
        else:
            return False
        

    def box_checked(self,new_waypoint_index):
        """ The indexes of a 1m^2 box around the new waypoint are added to the occupancy_data_np_checked"""

        for x in range(-5,5):
            for y in range(-5,5):
                slider= x * self.width + y
                self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, new_waypoint_index+slider)


    def cell_coordinates(self,index):
        """
        Given an index of a cell in the occupancy grid, it returns the coordinates of the cell in the map frame.
        """
        resolution = 0.05
        origin_x = self.current_grid.info.origin.position.x
        origin_y = self.current_grid.info.origin.position.y
        width = self.current_grid.info.width

        # Compute correspondent row and column of the potential cell
        row_index_float = index / width
        row_index = math.ceil(row_index_float)
        col_index = index % width

        # Compute position with respect map frame of the potential cell
        x_coord = (col_index*resolution) + origin_x
        y_coord = (row_index*resolution) + origin_y

        return x_coord, y_coord

    def current_position_callback(self, msg:PoseWithCovarianceStamped):
        #Return current robot pose, unless searching_for_waypoint
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id


    def readiness_check(self, msg:BehaviorTreeLog):
        """
        Call the next_waypoint() when specific conditions of the BehaviorTreeLog message are satisfied
        """

        for event in msg.event_log:
            

            if event.node_name == 'NavigateRecovery' and event.current_status =='IDLE':
                self.get_logger().info('NavigateRecovery--IDLE')
                self.next_waypoint()

    def image_callback(self, msg: Image):

        self.get_logger().info('Received image for ArUco detection')

        try:
            # 将ROS图像消息转换为OpenCV格式
            tag_size_in_meters = 0.1 
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 调试：显示原始相机图像
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # 用于更新图像显示，0为不延迟，1为1ms延迟显示

            # 转换为灰度图像
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # 验证 corners 是否检测到了标签
            if corners is not None:
                self.get_logger().info(f"Detected corners: {corners}")
            else:
                self.get_logger().error("No corners detected")
                return  # 如果没有检测到标签，提前退出
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                assert self.camera_matrix.shape == (3, 3), "camera_matrix 格式不正确"
                assert self.dist_coeffs.shape == (1, 5) or self.dist_coeffs.shape == (1, 4), "dist_coeffs 格式不正确"
            else:
                self.get_logger().error("Camera parameters not loaded properly.")
                return

            # 检测Aruco标签


            if ids is not None:
                # 如果检测到Aruco标签，打印标签ID
                self.get_logger().info(f"Detected ArUco marker(s) with ID(s): {ids}")

                # 可视化检测到的标签，显示在图像上
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.imshow("Detected ArUco Markers", cv_image)
                cv2.waitKey(1)  # 1ms延迟以更新窗口
                
                if self.camera_matrix is None or self.dist_coeffs is None:
                    self.get_logger().error("Camera parameters not loaded.")
                    return
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size_in_meters, self.camera_matrix, self.dist_coeffs)
                if rvecs is None or tvecs is None:
                    self.get_logger().error("Failed to estimate pose for ArUco marker")
                else:
                    for rvec, tvec in zip(rvecs, tvecs):
                        self.get_logger().info(f"Tag ID: {ids}, relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")

                for rvec, tvec in zip(rvecs, tvecs):
                # 计算标签相对于相机的坐标
                    self.get_logger().info(f"Tag ID: {ids}, relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")

                # 获取机器人当前的全局位置 (使用 self.current_position)
                    robot_x = self.current_position.point.x
                    robot_y = self.current_position.point.y
                    robot_yaw = 0  # 假设机器人没有角度变化，或者你可以通过机器人位姿获取 yaw 值

                # 计算全局坐标（相对于机器人位姿）
                    global_x = robot_x + (tvec[0][0] * math.cos(robot_yaw) - tvec[0][1] * math.sin(robot_yaw))
                    global_y = robot_y + (tvec[0][0] * math.sin(robot_yaw) + tvec[0][1] * math.cos(robot_yaw))
                    self.get_logger().info(f"Tag ID: {ids}, Global Position: x={global_x}, y={global_y}")

            else:
                # 如果未检测到标签，打印未检测到的消息
                self.get_logger().info("No ArUco markers detected")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
            
    def load_camera_parameters(self, file_path):
        self.get_logger().info(f"Loading camera parameters from: {file_path}")

        # 读取相机校准文件
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().error(f"Failed to open camera calibration file: {file_path}")
            return
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        
        if self.camera_matrix is not None and self.dist_coeffs is not None:
        # 打印出相机矩阵和畸变系数的信息
            self.get_logger().info(f"camera_matrix shape: {self.camera_matrix.shape}")
            self.get_logger().info(f"camera_matrix: {self.camera_matrix}")
            self.get_logger().info(f"dist_coeffs: {self.dist_coeffs}")
            assert self.camera_matrix.shape == (3, 3), "camera_matrix 格式不正确"
            assert self.dist_coeffs.shape == (1, 5) or self.dist_coeffs.shape == (1, 4), "dist_coeffs 格式不正确"
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().error("Failed to load camera parameters")
        else:
            self.get_logger().info(f"Loaded camera matrix: {self.camera_matrix}")
            self.get_logger().info(f"Loaded distortion coefficients: {self.dist_coeffs}")
            
            
            

    def select_points_for_rotation(self):
        self.get_logger().info("SLAM completed. Selecting points for rotation...")
        rotation_points = []

    # 遍历 occupancy grid 找到无障碍物的位置，添加到旋转点列表中
        occupancy_data_np = np.array(self.current_grid.data)
        for index in range(len(occupancy_data_np)):
            if occupancy_data_np[index] < self.obstacle_probability:  # 低于障碍概率的点认为是可行区域
                x, y = self.cell_coordinates(index)
                rotation_points.append((x, y))

    # 选取几个点进行旋转（这里简单取前 5 个点作为示例）
        selected_points = rotation_points[:5]
        self.get_logger().info(f"Selected {len(selected_points)} points for rotation.")

    # 依次移动到这些点，并在每个点旋转
        for point in selected_points:
            x, y = point
            self.move_to_point(x, y)
            self.rotate_robot_360()

    def rotate_robot_360(self):
        twist = Twist()
        twist.angular.z = 0.5  # 旋转速度
        rotate_duration = 2 * math.pi / twist.angular.z  # 计算旋转一圈的时间
        start_time = self.get_clock().now()

    # 开始旋转
        while (self.get_clock().now() - start_time).nanoseconds < rotate_duration * 1e9:
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Rotating...')

    # 停止旋转
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Rotation complete')
        
    def move_to_point(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # 假设没有角度变化

        self.waypoint_publisher.publish(goal_pose)
        self.get_logger().info(f"Moving to point: x={x}, y={y}")

    def on_slam_complete(self):

        self.get_logger().info("SLAM completed. Saving map and starting rotation checks.")

    # 保存地图
        self.save_map("/home/szx/ros2_ws/slam_map")

    # 选择点并进行旋转检测
        self.select_points_for_rotation()

    def save_map(self, filename="/home/szx/slam_map"):

        self.get_logger().info(f"Saving map to {filename}")
        try:
        # 使用 ROS 2 map_saver 命令行工具来保存地图
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', filename])
            self.get_logger().info(f"Map saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

        
def main():
    rclpy.init()
    autopilot_node = Autopilot()
    autopilot_node.get_logger().info('Running autopilot node')
    rclpy.spin(autopilot_node)

if __name__=='__main__':
    main()
