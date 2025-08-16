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
import yaml
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from collections import defaultdict
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory



class Aruco_detect(Node):

    def __init__(self):
        super().__init__('aruco_detect')
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_camera_parameters()

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
        self.aruco_positions = defaultdict(list)
        
        self.update_interval = 2.0  # Update navigation goal every 2 seconds
        self.navigation_timer = self.create_timer(self.update_interval, self.update_estimates)

        self.pos_queue_size=40

        self.received_image = False

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
            3
        )

        self.aruco_map_position_publisher = self.create_publisher(
            PointStamped,
            'aruco_map_position',
            3
        )       

        self.marker_publisher = self.create_publisher(
            MarkerArray, 
            'aruco_average_positions', 
            10
        )
    

    def current_position_callback(self, msg: PoseWithCovarianceStamped):

        # Exctact position (x, y)
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id

         # Extract orientation (quaternion)
        orientation_q = msg.pose.pose.orientation
        self.robot_orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    def image_callback(self, msg: Image):

        if not self.received_image:
            self.get_logger().info('Received image for ArUco detection')
            self.received_image = True

        try:
            # Convert ROS image message to OpenCV format
            tag_size_in_meters = 0.1
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            # Verify if corners detected the tag
            if len(corners)>0:
                self.get_logger().info(f"Detected corners: {corners}")


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

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size_in_meters, self.camera_matrix,
                                                                      self.dist_coeffs)
                if rvecs is None or tvecs is None:
                    self.get_logger().error("Failed to estimate pose for ArUco marker")
                    return

                for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                    marker_id = ids[i][0]  # Get the marker ID

                    # Check the number of position stored of the marker with the same ID
                    if len(self.aruco_positions[marker_id]) > self.pos_queue_size:
                        self.get_logger().info(f"Maximum number of measures reached for ID {marker_id} ")
                        continue
                    
                    self.get_logger().info(f"Tag ID: {marker_id}, Relative to camera: x={tvec[0][0]}, y={tvec[0][1]}, z={tvec[0][2]}")
                    
                    aruco = PointStamped()
                    aruco.header.frame_id = 'camera_rgb_optical_frame'
                    aruco.point.x = tvec[0][0]
                    aruco.point.y = tvec[0][1]
                    aruco.point.z = tvec[0][2]
                    self.aruco_position_publisher.publish(aruco)

                    try:
                        self.tf_buffer.can_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time())
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'camera_rgb_optical_frame',
                            rclpy.time.Time())
                        aruco_map_frame = tf2_geometry_msgs.do_transform_point(aruco, transform)
                        
                        self.get_logger().info(f"Tag ID: {marker_id}, Global Position: x={aruco_map_frame.point.x}, y={aruco_map_frame.point.y}, z={aruco_map_frame.point.z}")
                        
                        # Publish the transformed position
                        self.aruco_map_position_publisher.publish(aruco_map_frame)
                        

                        # Store the position in the dictionary
                        self.aruco_positions[marker_id].append(aruco_map_frame)
                    
                        
                    
                    except TransformException as ex:
                        self.get_logger().error(f"Could not transform tag position to map frame: {ex}")
                
                    time.sleep(1)

            else:
                pass

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


    def update_estimates(self):
        """
        Calculate the average position for each ArUco marker ID and publish for visualization
        """
        if not self.aruco_positions:
            self.get_logger().info("No ArUco marker positions to process")
            return

        marker_array = MarkerArray()

        for marker_id, positions in self.aruco_positions.items():
            if not positions:
                continue

            avg_x = sum(p.point.x for p in positions) / len(positions)
            avg_y = sum(p.point.y for p in positions) / len(positions)
            avg_z = sum(p.point.z for p in positions) / len(positions)

            avg_position = PointStamped()
            avg_position.header = positions[0].header
            avg_position.point.x = avg_x
            avg_position.point.y = avg_y
            avg_position.point.z = avg_z

            # Create a marker for this average position
            marker = Marker()
            marker.header = avg_position.header
            marker.ns = "aruco_average_positions"
            marker.id = int(marker_id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = avg_x
            marker.pose.position.y = avg_y
            marker.pose.position.z = avg_z
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

            self.get_logger().info(f"Average position for ArUco ID {marker_id}: x={avg_x:.2f}, y={avg_y:.2f}, z={avg_z:.2f}")

            # Publish the marker array for visualization
            self.marker_publisher.publish(marker_array)

    def load_camera_parameters(self):

        try:
            # Get share path and find the calibration file
            package_share_directory = get_package_share_directory('aruco_package')  
            calibration_file_path = os.path.join(package_share_directory, 'calibration_file.yaml')

            self.get_logger().info(f"Loading camera parameters from: {calibration_file_path}")
        except:
            self.get_logger().error(f"Error finding or loading camera parameters: {str(e)}")

        try:
            with open(calibration_file_path, 'r') as file:
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


def main():
    rclpy.init()
    aruco_detect_node = Aruco_detect()
    aruco_detect_node.get_logger().info('Running aruco detection node')
    rclpy.spin(aruco_detect_node)


if __name__ == '__main__':
    main()
