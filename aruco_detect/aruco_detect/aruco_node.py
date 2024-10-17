import numpy as np
import rclpy
import math
import time
from random import randrange
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import yaml
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

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

        self.robot_orientation = [0, 0, 0, 0]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dictionary to store ArUco marker positions
        self.aruco_positions = {}

        # Publisher for ArUco marker array
        self.aruco_array_publisher = self.create_publisher(
            MarkerArray,
            'aruco_marker_array',
            10
        )

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

        self.aruco_camera_position_publisher = self.create_publisher(
            PointStamped,
            'aruco_camera_position',
            self.queue_size
        )

        self.aruco_map_position_publisher = self.create_publisher(
            PointStamped,
            'aruco_map_position',
            self.queue_size
        )
    
    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert a quaternion to yaw angle (rotation around Z axis)
        """
        # Calculate yaw (z-axis rotation) from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def current_position_callback(self, msg: PoseWithCovarianceStamped):

        # Exctact position (x, y)
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id

         # Extract orientation (quaternion)
        orientation_q = msg.pose.pose.orientation
        self.robot_orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

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

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size_in_meters, self.camera_matrix,
                                                                      self.dist_coeffs)
                if rvecs is None or tvecs is None:
                    self.get_logger().error("Failed to estimate pose for ArUco marker")

                for revcs, tvec in zip(rvecs, tvecs):
                    # Calculate the tag coordinates relative to the camera
                    self.get_logger().info(
                        f"Tag ID: {ids}, relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")

                    # Create a PointStamped message for the ArUco tag position
                    aruco_camera_frame = PointStamped()
                    aruco_camera_frame.header.frame_id = 'camera_rgb_optical_frame'
                    aruco_camera_frame.header.stamp = self.get_clock().now().to_msg()
                    aruco_camera_frame.point.x = tvec[0][0]
                    aruco_camera_frame.point.y = tvec[0][1]
                    aruco_camera_frame.point.z = tvec[0][2]

                    self.aruco_camera_position_publisher.publish(aruco_camera_frame)

                    try:
                        # Wait for the transform to be available
                        self.tf_buffer.can_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time())
                        
                        # Transform the point from camera frame to map frame
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'camera_rgb_optical_frame',
                            rclpy.time.Time())
                        aruco_map_frame = tf2_geometry_msgs.do_transform_point(aruco_camera_frame, transform)
                        
                        self.get_logger().info(f"Tag ID: {ids}, Global Position: x={aruco_map_frame.point.x}, y={aruco_map_frame.point.y}, z={aruco_map_frame.point.z}")
                        

                            # Store or update the position of this ArUco marker
                        """ marker_id = ids[i]
                         self.aruco_positions[marker_id] = aruco_map_frame.point"""

                       
                        
                        # Publish the transformed position
                        self.aruco_map_position_publisher.publish(aruco_map_frame)
                        
                    except TransformException as ex:
                        self.get_logger().error(f"Could not transform tag position to map frame: {ex}")

                    time.sleep(5)

            else:
                # If no tags detected, print message
                self.get_logger().info("No ArUco markers detected")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


            

    def load_camera_parameters(self, file_path):
        self.get_logger().info(f"Loading camera parameters from: {file_path}")

        try:
            with open(file_path, 'r') as file:
                camera_data = yaml.safe_load(file)

                self.camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape((3, 3))
                self.dist_coeffs = np.array(camera_data['distortion_coefficients']['data']).reshape((1, 5))
        except Exception as e:
            self.get_logger().error(f"Error opening parameters file: {str(e)}")
    

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



    def publish_aruco_marker_array(self):
        marker_array = MarkerArray()
        
        for marker_id, point in self.aruco_positions.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "aruco_markers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.aruco_array_publisher.publish(marker_array)

def main():
    rclpy.init()
    aruco_detect_node = Aruco_detect()
    aruco_detect_node.get_logger().info('Running aruco detection node')
    rclpy.spin(aruco_detect_node)


if __name__ == '__main__':
    main()
