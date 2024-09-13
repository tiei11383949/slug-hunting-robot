from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
        
    # Parameters, Nodes and Launch files go here
    # Declare package directory
    pkg_demos = get_package_share_directory('octomap_mapping')
    exp_demos = get_package_share_directory('explore_lite')
    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([pkg_demos, 'omni_bt_navigator.yaml'])
    config_planner    = PathJoinSubstitution([pkg_demos, 'omni_planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_demos, 'omni_controller.yaml'])
    config_behaviour  = PathJoinSubstitution([pkg_demos, 'omni_behaviour.yaml']) 
    config_explore    = PathJoinSubstitution([exp_demos, 'config', 'params.yaml'])
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
    ]
    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    node_tf = Node(
        ## Configure the TF of the robot to the origin of the map coordinates
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom_orb']
    )
    
    node_footprint = Node(
             ## Configure the TF of the robot to the origin of the map coordinates
             package='tf2_ros',
             executable='static_transform_publisher',
             output='screen',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )
        
    node_octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,  # Octomap resolution
            'frame_id': 'map',  # Reference frame
        }],
        remappings=[
            ('/cloud_in', '/orbslam3/points')  # Replace with your actual topic
        ]
    )
    
    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_behaviour],
        remappings=remappings,
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner],
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_controller],
        remappings=remappings,
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )
    
    node_explore = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        output="screen",
        parameters=[config_explore],
        remappings=remappings,
    )

    ld.add_action(node_tf)
    ld.add_action(node_footprint)
    ld.add_action(node_octomap)
    
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_lifecycle_manager)
    
    #ld.add_action(node_explore)
    return ld

