from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slip_monitor',
            executable='slip_monitor_node',
            name='slip_monitor_node',
            output='screen',
            parameters=[{
                'wheel.diameter': 0.254,
                'wheel.tread': 0.60,
                'output_dir': '.',
            }]
        )
    ])
