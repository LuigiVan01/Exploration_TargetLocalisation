from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot_physical_package',
            executable='aruco_detect_robot',
            name='aruco_node_robot',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
