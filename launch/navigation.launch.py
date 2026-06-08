import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    stage_dir = get_package_share_directory('stage_ros2_stageros')

    map_yaml = os.path.join(pkg_dir, 'map', 'map.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'navigation_rviz.rviz')
    world_file = os.path.join(pkg_dir, 'worlds', 'second_project.world')
    stage_launch = os.path.join(stage_dir, 'launch', 'stage.launch.py')
    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('stage_gui'),
            'use_model_names': 'false',
            'base_watchdog_timeout': '0.2',
            'delay_odom_tf_by_one_update': 'true',
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'namespace': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map'),
            },
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'global_frame_id': LaunchConfiguration('map_frame'),
                'set_initial_pose': ParameterValue(
                    LaunchConfiguration('set_initial_pose'), value_type=bool),
                'always_reset_initial_pose': ParameterValue(
                    LaunchConfiguration('set_initial_pose'), value_type=bool),
                'initial_pose': {
                    'x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                    'y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                    'z': 0.0,
                    'yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
                },
            },
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server', 'amcl'],
        }],
    )

    goal_publisher = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='second_project',
                executable='goal_publisher',
                name='goal_publisher',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_id': LaunchConfiguration('map_frame'),
                    'action_name': 'navigate_to_pose',
                }],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_goal_publisher')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('world', default_value=world_file, description='Stage world file.'),
        DeclareLaunchArgument('stage_gui', default_value='true', description='Start the Stage GUI.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use /clock from Stage.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Auto-start lifecycle nodes.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('use_goal_publisher', default_value='true', description='Send CSV goals automatically.'),
        DeclareLaunchArgument('map', default_value=map_yaml, description='Occupancy map YAML file.'),
        DeclareLaunchArgument('scan_topic', default_value='/base_scan', description='Stage LaserScan topic.'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame.'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Global map frame.'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint', description='Robot base frame.'),
        DeclareLaunchArgument('set_initial_pose', default_value='true', description='Initialize AMCL from launch args.'),
        DeclareLaunchArgument('initial_x', default_value='1.6', description='Initial robot x in map frame.'),
        DeclareLaunchArgument('initial_y', default_value='-4.4', description='Initial robot y in map frame.'),
        DeclareLaunchArgument('initial_yaw', default_value='-0.013', description='Initial robot yaw in radians.'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params, description='Nav2 params file.'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config, description='RViz config file.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level.'),
        stage,
        map_server,
        amcl,
        lifecycle_manager_localization,
        nav2_navigation,
        goal_publisher,
        rviz,
    ])
