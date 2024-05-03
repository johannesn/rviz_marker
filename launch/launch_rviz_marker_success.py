from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz_marker',
            executable='marker_array',
            name='marker_array',
            parameters=[{
                "frame": "odom"
            }]
        ),
        Node(
            package='rviz_marker',
            executable='marker',
            name='marker',
            parameters=[{
                "frame": "odom"
            }]
        ),
        Node(
            package='rviz_marker',
            executable='base_link',
            name='base_link'
        )
    ])
