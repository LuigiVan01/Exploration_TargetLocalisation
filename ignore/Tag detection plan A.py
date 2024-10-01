import numpy as np
import rclpy
import math
import cv2
import cv2.aruco as aruco
from random import randrange
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Autopilot(Node):

    def __init__(self):
        super().__init__('autopilot')

        # Allow callback functions to be called in parallel
        self.parallel_callback_group = ReentrantCallbackGroup()

        # Initialize the CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the camera image topic (assumes you have a camera publishing to '/camera/rgb/image_raw')
        self.camera_subscriber = self.create_subscription(Image, '/camera/rgb/image_raw', self.camera_callback, 10, callback_group=self.parallel_callback_group)

        # Create a publisher to control the robot's velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variables for ArUco marker detection
        self.aruco_detected_ids = set()  # Store the detected ArUco marker IDs

        # Initialize the other variables for navigation and grid exploration
        self.obstacle_probability = 90
        self.queue_size = 10
        self.start = True
        self.waypoint_counter = 0
        self.occupancy_data_np_checked = []
        self.current_grid = OccupancyGrid()
        self.previous_grid = OccupancyGrid()
        self.difference_grid = OccupancyGrid()
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0
        self.potential_coordinate = PointStamped()
        self.potential_coordinate.header.frame_id = 'map'
        self.potential_coordinate.point.x = 0.0
        self.potential_coordinate.point.y = 0.0
        self.current_position = PointStamped()
        self.current_position.header.frame_id = 'map'
        self.strategy_counter = 10

        # Subscribe to /behavior_tree_log to determine when TurtleBot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(
            BehaviorTreeLog, 'behavior_tree_log', self.readiness_check, self.queue_size, callback_group=self.parallel_callback_group)

        # Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.store_grid, self.queue_size)

        # Subscribe to /pose to determine position of TurtleBot
        self.position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.current_position_callback, self.queue_size, callback_group=self.parallel_callback_group)

        # Create publishers to publish next waypoint and debug information
        self.waypoint_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', self.queue_size)
        self.potential_publisher = self.create_publisher(
            PointStamped, 'potential_point', self.queue_size)
        self.current_publisher = self.create_publisher(
            PointStamped, 'current_point', self.queue_size)

        # Track number of waypoints sent
        self.waypoint_counter = 0.0

    def camera_callback(self, image_msg):
        """Process the incoming camera images to detect ArUco markers."""
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Convert the image to grayscale for ArUco detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Load the predefined dictionary for ArUco markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        # Detect the markers in the image
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are found, process them
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.aruco_detected_ids:
                    self.get_logger().info(f"ArUco marker detected! ID: {marker_id}")
                    self.aruco_detected_ids.add(marker_id)  # Add detected marker ID to the set

                    # Draw detected markers on the image
                    aruco.drawDetectedMarkers(cv_image, corners)

                    # Calculate the marker's center
                    marker_center_x = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                    marker_center_y = (corners[i][0][0][1] + corners[i][0][2][1]) / 2

                    # Calculate position relative to the robot
                    robot_x = self.current_position.point.x
                    robot_y = self.current_position.point.y
                    distance = math.sqrt((marker_center_x - robot_x) ** 2 + (marker_center_y - robot_y) ** 2)
                    self.get_logger().info(f"Marker ID: {marker_id}, Relative distance: {distance:.2f}")

                    # Perform action based on detected marker
                    self.perform_action()

        # Optionally show the image with detected markers
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def perform_action(self):
        """Perform a simple action when an ArUco marker is detected."""
        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        twist.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Moving robot forward as ArUco marker was detected!")

    def store_grid(self, grid: OccupancyGrid):
        """Callback function of /map topic."""
        self.previous_grid = self.current_grid
        self.current_grid = grid
        self.get_logger().info('Grid Received')

        if self.start:
            self.start = False
            self.next_waypoint()

    def next_waypoint(self):
        """Function to choose next waypoint when new occupancy grid is received."""
        self.width = self.current_grid.info.width
        isthisagoodwaypoint = False
        not_in_range_count = 0
        min_distance = 1
        max_distance = 3
        still_looking = False
        occupancy_data_np = np.array(self.current_grid.data)

        if self.strategy_counter > 0:
            while not isthisagoodwaypoint:
                if not still_looking:
                    self.get_logger().info('Searching for good point...')
                    still_looking = True

                random_index = randrange(occupancy_data_np.size)
                self.potential_pos = occupancy_data_np[random_index]

                if random_index in self.occupancy_data_np_checked or self.potential_pos == -1:
                    continue

                [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(random_index)
                self.potential_publisher.publish(self.potential_coordinate)
                self.current_publisher.publish(self.current_position)

                if self.potential_pos >= self.obstacle_probability:
                    self.get_logger().info(f'Point was an obstacle with cost: {self.potential_pos}')
                    self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, random_index)
                    continue

                elif not self.frontier_check(occupancy_data_np, random_index):
                    self.get_logger().info('Point was not on frontier')
                    self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, random_index)
                    continue

                else:
                    self.get_logger().info('Found Good Point')
                    distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.point.x) ** 2 +
                        (self.potential_coordinate.point.y - self.current_position.point.y) ** 2)

                    if min_distance < distance2new < max_distance:
                        self.new_waypoint.pose.position.x = self.potential_coordinate.point.x
                        self.new_waypoint.pose.position.y = self.potential_coordinate.point.y
                        isthisagoodwaypoint = True
                        self.box_checked(random_index)
                        self.strategy_counter -= 1

                    else:
                        self.get_logger().info('Point not in range, Finding New...')
                        still_looking = False
                        not_in_range_count += 1
                        if not_in_range_count > 50:
                            self.get_logger().
                                                        self.get_logger().info('Could not find point in range, adopting new strategy...')
                            self.new_strategy()
                            isthisagoodwaypoint = True

        # New strategy if no good points are found in the previous method
        else:
            self.strategy_counter = 5
            self.new_strategy()

        # Publish the new waypoint
        self.get_logger().info('Publishing waypoint...')
        self.waypoint_publisher.publish(self.new_waypoint)

    def new_strategy(self):
        """Processes the occupancy grid and creates a sorted list of cells based on the number of uncertain cells around them."""
        self.get_logger().info('New Strategy: Processing occupancy grid ...')
        occupancy_data_np = np.array(self.current_grid.data)
        width = self.current_grid.info.width
        counts_list = []

        # Iterate over all cells in the occupancy grid
        for index in range(len(occupancy_data_np)):
            if index in self.occupancy_data_np_checked or occupancy_data_np[index] == -1:
                continue

            elif occupancy_data_np[index] >= self.obstacle_probability:
                self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, index)
                continue

            else:
                # Count the number of uncertain cells around the current cell and append to the list
                uncertain_count = self.count_uncertain_cells_around(index, occupancy_data_np, width)
                counts_list.append((index, uncertain_count))

        # Sort the list by uncertain_count in descending order
        sorted_counts = sorted(counts_list, key=lambda x: x[1], reverse=True)

        # Take the cell with the most uncertain surrounding cells and calculate its coordinates
        [self.new_waypoint.pose.position.x, self.new_waypoint.pose.position.y] = self.cell_coordinates(sorted_counts[0][0])
        self.box_checked(sorted_counts[0][0])
        [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(sorted_counts[0][0])
        self.potential_publisher.publish(self.potential_coordinate)

    def count_uncertain_cells_around(self, index, occupancy_data_np, width):
        """Counts the number of uncertain cells (-1) around a given cell within a defined box size."""
        uncertain_count = 0
        box_size = 5  # Define the size of the box around each cell

        # Loop over the box around the cell
        for x in range(-box_size, box_size + 1):
            for y in range(-box_size, box_size + 1):
                slider = x * width + y
                try:
                    if occupancy_data_np[index + slider] == -1:
                        uncertain_count += 1
                except IndexError:
                    continue
        return uncertain_count

    def frontier_check(self, occupancy_data_np, random_index):
        """Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles."""
        uncertain_indexes = 0
        obstacle_indexes = 0

        # Inspects the nature of points in a grid around the selected point
        for x in range(-9, 10):
            for y in range(-9, 10):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[random_index + slider] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data_np[random_index + slider] > self.obstacle_probability:
                        obstacle_indexes += 1
                except IndexError:
                    continue

        if uncertain_indexes > 1:
            return True
        else:
            return False

    def box_checked(self, new_waypoint_index):
        """The indexes of a 1m^2 box around the new waypoint are added to the occupancy_data_np_checked."""
        for x in range(-9, 10):
            for y in range(-9, 10):
                slider = x * self.width + y
                self.occupancy_data_np_checked = np.append(self.occupancy_data_np_checked, new_waypoint_index + slider)

    def cell_coordinates(self, index):
        """Given an index of a cell in the occupancy grid, it returns the coordinates of the cell in the map frame."""
        resolution = 0.05
        origin_x = self.current_grid.info.origin.position.x
        origin_y = self.current_grid.info.origin.position.y
        width = self.current_grid.info.width

        # Compute correspondent row and column of the potential cell
        row_index_float = index / width
        row_index = math.ceil(row_index_float)
        col_index = index % width

        # Compute position with respect map frame of the potential cell
        x_coord = (col_index * resolution) + origin_x
        y_coord = (row_index * resolution) + origin_y

        return x_coord, y_coord

    def current_position_callback(self, msg: PoseWithCovarianceStamped):
        """Callback function for current robot pose."""
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id

    def readiness_check(self, msg: BehaviorTreeLog):
        """Call the next_waypoint() when specific conditions of the BehaviorTreeLog message are satisfied."""
        for event in msg.event_log:
            if event.node_name == 'NavigationRecovery' and event.current_status == 'IDLE':
                self.next_waypoint()

            elif event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.next_waypoint()

            elif event.node_name == 'GoalUpdated' and event.current_status == "FAILURE":
                self.next_waypoint()

def main():
    rclpy.init()
    autopilot_node = Autopilot()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(autopilot_node)
    autopilot_node.get_logger().info('Running autopilot node')
    executor.spin()
    autopilot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
