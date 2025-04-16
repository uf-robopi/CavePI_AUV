from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch detector node
        Node(
            package='AUV_nano',
            executable='detector',
            name='detector',
            output='screen',
            parameters=[{
            }]
        ),

        # Launch planner node
        Node(
            package='AUV_nano',
            executable='planner',
            name='planner',
            output='screen',
            parameters=[{
            }]
        )
    ])
