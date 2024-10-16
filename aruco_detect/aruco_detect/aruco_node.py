import numpy as np
import rclpy
import math
import time
from random import randrange
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Aruco_detect(Node):

    def __init__(self):
        super().__init__('autopilot')
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_camera_parameters("/home/lv/turtlebot3_ws/src/aruco_detect/calibration_file.yaml")

        self.queue_size = 10

        # Initialize cv_bridge
        self.bridge = CvBridge()

        # Initialize aruco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Initializing current position variable
        self.current_position = PointStamped()
        self.current_position.header.frame_id = 'map'

        # Subscribe to /pose to determine the position of Turtlebot
        self.position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.current_position_callback,
            self.queue_size,
            # callback_group=self.parallel_callback_group
        )

        # Subscribe to image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            self.queue_size
        )

        self.aruco_position_publisher = self.create_publisher(
            PointStamped,
            'aruco_position',
            self.queue_size
        )

    def current_position_callback(self, msg: PoseWithCovarianceStamped):
        # Return current robot pose, unless searching for waypoint
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id

    def image_callback(self, msg: Image):
        self.get_logger().info('Received image for ArUco detection')

        try:
            # Convert ROS image message to OpenCV format
            tag_size_in_meters = 0.1
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            # Verify if corners detected the tag
            if corners is not None:
                self.get_logger().info(f"Detected corners: {corners}")
            else:
                self.get_logger().error("No corners detected")
                return  # Exit early if no tags detected
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                assert self.camera_matrix.shape == (3, 3), "Incorrect format for camera_matrix"
                assert self.dist_coeffs.shape == (1, 5) or self.dist_coeffs.shape == (
                1, 4), "Incorrect format for dist_coeffs"
            else:
                self.get_logger().error("Camera parameters not loaded properly.")
                return

            # Detect Aruco tags
            if ids is not None:
                # If Aruco tags detected, print the tag IDs
                self.get_logger().info(f"Detected ArUco marker(s) with ID(s): {ids}")

                # Visualize the detected tags, display on the image
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.imshow("Detected ArUco Markers", cv_image)
                cv2.waitKey(1)  # 1ms delay to update the window

                if self.camera_matrix is None or self.dist_coeffs is None:
                    self.get_logger().error("Camera parameters not loaded.")
                    return

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size_in_meters, self.camera_matrix,
                                                                      self.dist_coeffs)
                if rvecs is None or tvecs is None:
                    self.get_logger().error("Failed to estimate pose for ArUco marker")
                else:
                    for rvec, tvec in zip(rvecs, tvecs):
                        self.get_logger().info(
                            f"Tag ID: {ids}, relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")

                for rvec, tvec in zip(rvecs, tvecs):
                    # Calculate the tag coordinates relative to the camera
                    self.get_logger().info(
                        f"Tag ID: {ids}, relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")

                    # Get robot's current global position (using self.current_position)
                    robot_x = self.current_position.point.x
                    robot_y = self.current_position.point.y
                    robot_yaw = 0  # Assume no rotation for the robot, or you can obtain the yaw from the robot pose

                    # Calculate global coordinates (relative to robot pose)
                    global_x = robot_x + tvec[0][0]
                    global_y = robot_y + tvec[0][1]
                    # global_x = robot_x + (tvec[0][0] * math.cos(robot_yaw) - tvec[0][1] * math.sin(robot_yaw))
                    # global_y = robot_y + (tvec[0][0] * math.sin(robot_yaw) + tvec[0][1] * math.cos(robot_yaw))
                    self.get_logger().info(f"Tag ID: {ids}, Global Position: x={global_x}, y={global_y}")
                    aruco = PointStamped()
                    aruco.header.frame_id = 'map'
                    aruco.point.x = global_x
                    aruco.point.y = global_y
                    self.aruco_position_publisher.publish(aruco)
                    time.sleep(1)

            else:
                # If no tags detected, print message
                self.get_logger().info("No ArUco markers detected")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


            

    def load_camera_parameters(self, file_path):
        self.get_logger().info(f"Loading camera parameters from: {file_path}")

        # Read camera calibration file
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().error(f"Failed to open camera calibration file: {file_path}")
            return
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Print information about the camera matrix and distortion coefficients
            self.get_logger().info(f"camera_matrix shape: {self.camera_matrix.shape}")
            self.get_logger().info(f"camera_matrix: {self.camera_matrix}")
            self.get_logger().info(f"dist_coeffs: {self.dist_coeffs}")
            assert self.camera_matrix.shape == (3, 3), "Incorrect format for camera_matrix"
            assert self.dist_coeffs.shape == (1, 5) or self.dist_coeffs.shape == (
            1, 4), "Incorrect format for dist_coeffs"
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().error("Failed to load camera parameters")
        else:
            self.get_logger().info(f"Loaded camera matrix: {self.camera_matrix}")
            self.get_logger().info(f"Loaded distortion coefficients: {self.dist_coeffs}")


def main():
    rclpy.init()
    aruco_detect_node = Aruco_detect()
    aruco_detect_node.get_logger().info('Running aruco detection node')
    rclpy.spin(aruco_detect_node)


if __name__ == '__main__':
    main()
