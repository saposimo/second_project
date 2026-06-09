import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _launch_stage_with_scan_relay(context, *args, **kwargs):
    stage_args = []
    if not _truthy(LaunchConfiguration('stage_gui').perform(context)):
        stage_args.append('-g')
    stage_args.append(LaunchConfiguration('world').perform(context))

    stage = Node(
        package='stage_ros2_stageros',
        executable='stageros',
        name='stageros',
        output='screen',
        arguments=stage_args,
        parameters=[{
            'base_watchdog_timeout': 0.2,
            'is_depth_canonical': True,
            'use_model_names': False,
            'delay_odom_tf_by_one_update': True,
            # Throttle Stage to wall-clock time even when headless (no GUI to pace it).
            'realtime_factor': 1.0,
            'use_sim_time': _truthy(LaunchConfiguration('use_sim_time').perform(context)),
        }],
        remappings=[
            ('base_scan', 'base_scan_raw'),
        ],
    )

    scan_relay = Node(
        package='second_project',
        executable='scan_qos_relay',
        name='stage_scan_qos_relay',
        output='screen',
        parameters=[{
            'input_topic': '/base_scan_raw',
            'output_topic': LaunchConfiguration('scan_topic').perform(context),
        }],
    )

    return [stage, scan_relay]


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    map_yaml = os.path.join(pkg_dir, 'map', 'map.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'navigation_rviz.rviz')
    world_file = os.path.join(pkg_dir, 'worlds', 'second_project.world')
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    controller_remappings = nav2_remappings + [('cmd_vel', 'cmd_vel_nav')]
    velocity_smoother_remappings = nav2_remappings + [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel'),
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=controller_remappings,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=nav2_remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=nav2_remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=controller_remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=nav2_remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=velocity_smoother_remappings,
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

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ],
        }],
    )

    goal_publisher = TimerAction(
        period=12.0,
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
        DeclareLaunchArgument('stage_gui', default_value='false', description='Start the Stage GUI.'),
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
        OpaqueFunction(function=_launch_stage_with_scan_relay),
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
        goal_publisher,
        rviz,
    ])
