from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='services_quiz',
            executable='service_client',
            name='turn_service_client',
            output='screen'
        )
    ])
