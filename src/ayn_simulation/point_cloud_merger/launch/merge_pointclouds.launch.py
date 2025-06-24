from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_merger',
            executable='merge_pointclouds.py',
            name='point_cloud_merger',
            parameters=[{
                'input_topics': ['/camera_front/points', '/camera_left/points', '/camera_right/points'],
                'output_topic': '/merged_points',
                'target_frame': 'base_link',
            }]
        )
    ])