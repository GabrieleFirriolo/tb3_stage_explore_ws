from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction
def generate_launch_description():

    # Launch arguments
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')


    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='turtlebot'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            FindPackageShare('stage_ros2').find('stage_ros2'), 'world', 'cave.world')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            FindPackageShare('tb3_stage_explore').find('tb3_stage_explore'), 'params', 'nav2_params.yaml')),
    ]


    # Stage simulator
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,'world_file':world_path}],
    )


    # GMapping SLAM
    # Parameters not getting used in ros2_gmapping
    slam_node = Node(
        package='gmapper',
        executable='gmap',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/scan', '/base_scan')],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )

    # Nav2 bringup (Nav stack)
    nav2_bringup_dir = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch')

    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup_dir, 'bringup_launch.py')
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'slam': 'True',                    
        'use_localization': 'False',       
        'log_level':'ERROR'
    }.items()
)
    
    

    #Explore Lite
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/map',    # <<<<<<<<<<<
            'costmap_updates_topic': '/map_updates',   # <<<<<<<<<<<
            'visualize': True,
            'planner_frequency': 0.05,#0.1, # 0.05
            'progress_timeout': 80.0,#40.0, # 80.0
            'potential_scale': 2.0,
            'gain_scale': 0.0,
            'transform_tolerance': 1.0,
            'min_frontier_size': 0.8,
            'output':'log'
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ])

    # Explore Lite, avviato con un piccolo delay
    # delayed_explore_node = TimerAction(
    #     period=7.0,  # <-- secondi di ritardo (puoi aumentare/diminuire)
    #     actions=[explore_node]
    # )
    # delayed_nav2 = TimerAction(
    #     period=3.0,  # <-- secondi di ritardo (puoi aumentare/diminuire)
    #     actions=[nav2_launch]
    # )

    return LaunchDescription(declare_args + [
        stage_node,
        slam_node,
        nav2_launch,
        static_tf,
        explore_node
    ])
