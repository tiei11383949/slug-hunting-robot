from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
        Node(
             ## Configure the TF of the robot to the origin of the map coordinates
             package='tf2_ros',
             executable='static_transform_publisher',
             output='screen',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
         ),
         
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.05,  # Octomap resolution
                'frame_id': 'odom',  # Reference frame
            }],
            remappings=[
                ('/cloud_in', '/bot/rgbd/points')  # Replace with your actual topic
            ]
        ),
    ])
