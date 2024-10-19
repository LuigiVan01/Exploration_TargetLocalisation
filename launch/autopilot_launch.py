from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot',
            executable='autopilot',
            name='autopilot',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
