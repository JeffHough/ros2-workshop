from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub',
            executable='minimal_publisher',
        ),
        Node(
            package='pubsub',
            executable='minimal_subscriber.py',
        ),
    ])
