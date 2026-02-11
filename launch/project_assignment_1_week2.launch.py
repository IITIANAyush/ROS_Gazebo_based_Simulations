import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gauntlet = get_package_share_directory('gauntlet_sim')
    
    # ---------------- Launch Arguments ----------------
    # This allows you to use: planner_type:=astar or planner_type:=gvd
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='astar',
        description='Which follower to run: astar or gvd'
    )

    planner_type_config = LaunchConfiguration('planner_type')

    # 1. Path to your Map YAML
    map_yaml_file = '/home/ayush/Desktop/IITB_Acads/6th_Sem/SC627/Assign_1_sol/maps/gauntlet_map.yaml'

    # 2. Include the Gazebo + Robot Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gauntlet, 'launch', 'gauntlet_tb3.launch.py')
        )
    )

    # 3. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Static TF Publisher (Map -> Odom)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # 5. Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': True
        }]
    )

    # 6. Lifecycle Manager
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 7. Planner Server (Delayed by 10 seconds)
    planner_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='Launching Planner Server...'),
            Node(
                package='gauntlet_sim',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 8. Path Followers (Conditional Actions)
    
    # A* Follower Node (Points to path_follower_A_star via setup.py key)
    astar_follower_node = Node(
        condition=LaunchConfigurationEquals('planner_type', 'astar'),
        package='gauntlet_sim',
        executable='path_follower',
        name='path_follower_astar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # GVD Follower Node (Points to path_follower_GVD via setup.py key)
    gvd_follower_node = Node(
        condition=LaunchConfigurationEquals('planner_type', 'gvd'),
        package='gauntlet_sim',
        executable='path_follower_gvd',
        name='path_follower_gvd',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Wrap followers in a TimerAction (Delayed by 12 seconds)
    follower_timer = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=['Starting follower for: ', planner_type_config]),
            astar_follower_node,
            gvd_follower_node
        ]
    )

    return LaunchDescription([
        planner_type_arg,
        gazebo_launch,
        rviz_node,
        static_tf,
        map_server_node,
        map_lifecycle_manager,
        planner_node,
        follower_timer
    ])
