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
        self.aruco_detected = False

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

        # If markers are found, print their IDs and take action
        if ids is not None:
            self.get_logger().info(f"ArUco marker detected! IDs: {ids.flatten()}")
            self.aruco_detected = True

            # Execute a simple action, such as moving the robot
            self.perform_action()

        # Optionally show the image with detected markers
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def perform_action(self):
        """Perform a simple action when an ArUco marker is detected."""
        if self.aruco_detected:
            # Create a Twist message to move the robot
            twist = Twist()
            twist.linear.x = 0.2  # Move forward at 0.2 m/s
            twist.angular.z = 0.0  # No rotation

            # Publish the movement command
            self.cmd_vel_publisher.publish(twist)

            self.get_logger().info("Moving robot forward as ArUco marker was detected!")

            # Stop the robot after a short delay (simulating a reaction to the tag detection)
            self.aruco_detected = False


def main():
    rclpy.init()
    autopilot_node = Autopilot()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(autopilot_node)
    executor.spin()
    autopilot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
