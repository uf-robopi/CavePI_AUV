from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    data_receiver_node = Node(
        package='cavepi_controller',
        executable='data_receiver',
        name='data_receiver',
        output='screen',
    )

    autopliot_node = Node(
        package='cavepi_controller',
        executable='autopilot',
        name='autopilot',
        output='screen'
    )

    autopilot_util_node = Node(
        package='cavepi_controller',
        executable='autopilot_util',
        name='autopilot_util',
        output='screen'
    )
    
    ping_publisher_node = Node(
        package='cavepi_controller',
        executable='ping2_publisher',
        name='ping2_publisher',
        output='screen',
    )
    
    ld.add_action(data_receiver_node)
    ld.add_action(autopliot_node)
    ld.add_action(autopilot_util_node)    
    ld.add_action(ping_publisher_node)

    return ld
