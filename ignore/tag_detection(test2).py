import rclpy
import math
import cv2
import cv2.aruco as aruco
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError


class ArucoTagDetector(Node):

    def __init__(self):
        super().__init__('aruco_tag_detector')

        # 使用 ReentrantCallbackGroup 允许并行执行回调函数
        self.parallel_callback_group = ReentrantCallbackGroup()

        # 初始化 CvBridge 用于将 ROS Image 消息转换为 OpenCV 图像
        self.bridge = CvBridge()

        # 已检测到的 ArUco 标记的集合
        self.detected_aruco_ids = set()

        # 订阅摄像头图像话题 (假设摄像头发布在 '/camera/rgb/image_raw')
        self.camera_subscriber = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10, callback_group=self.parallel_callback_group)

        # 订阅机器人位置信息 (Pose) 用于计算相对 ArUco 标记的距离
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.robot_pose_callback, 10, callback_group=self.parallel_callback_group)

        # 初始化机器人的当前位置
        self.robot_x = 0.0
        self.robot_y = 0.0

    def robot_pose_callback(self, msg: PoseWithCovarianceStamped):
        """从机器人位姿回调函数中获取机器人的位置。"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.get_logger().info(f"Robot position updated: x = {self.robot_x}, y = {self.robot_y}")

    def camera_callback(self, image_msg):
        """处理从摄像头接收到的图像并检测 ArUco 标记。"""
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # 将彩色图像转换为灰度图像
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 加载 ArUco 字典和检测参数
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        aruco_parameters = aruco.DetectorParameters_create()

        # 检测图像中的 ArUco 标记
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_parameters)

        # 如果检测到标记，处理每个标记
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.detected_aruco_ids:
                    self.get_logger().info(f"ArUco marker detected! ID: {marker_id}")
                    # 将检测到的标记 ID 添加到已检测集合中，避免重复检测
                    self.detected_aruco_ids.add(marker_id)

                    # 在图像上绘制检测到的 ArUco 标记
                    aruco.drawDetectedMarkers(cv_image, corners)

                    # 计算标记的中心
                    marker_center_x = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                    marker_center_y = (corners[i][0][0][1] + corners[i][0][2][1]) / 2

                    # 计算标记相对于机器人的距离
                    relative_distance = self.calculate_relative_distance(marker_center_x, marker_center_y)
                    self.get_logger().info(f"Marker ID: {marker_id}, Relative distance: {relative_distance:.2f}")

        # 显示带有检测到的标记的图像
        cv2.imshow("ArUco Marker Detection", cv_image)
        cv2.waitKey(1)

    def calculate_relative_distance(self, marker_x, marker_y):
        """计算标记相对于机器人的距离。"""
        distance = math.sqrt((marker_x - self.robot_x) ** 2 + (marker_y - self.robot_y) ** 2)
        return distance


def main():
    rclpy.init()

    # 创建 ArucoTagDetector 节点
    aruco_detector = ArucoTagDetector()

    # 使用多线程执行器以并行处理回调
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(aruco_detector)

    # 运行节点
    try:
        aruco_detector.get_logger().info('ArucoTagDetector node is running...')
        executor.spin()
    finally:
        # 停止并销毁节点
        aruco_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
