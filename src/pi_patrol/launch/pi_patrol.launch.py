from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch all nodes for the Pi Patrol system."""
    
    return LaunchDescription([
        Node(
            package='pi_patrol',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='pi_patrol',
            executable='detection_node',
            name='detection_node',
            output='screen',
        ),
        Node(
            package='pi_patrol',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
        ),
        Node(
            package='pi_patrol',
            executable='telegram_notifier_node',
            name='telegram_notifier_node',
            output='screen',
        ),
    ])
