from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_package',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
