import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir        = get_package_share_directory('second_project')
    nav2_bringup   = get_package_share_directory('nav2_bringup')

    map_yaml    = os.path.join(pkg_dir, 'map',    'map.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'navigation_rviz.rviz')
    world_file  = os.path.join(pkg_dir, 'worlds', 'second_project.world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_goal_publisher = LaunchConfiguration('use_goal_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the navigation stack.',
        ),
        DeclareLaunchArgument(
            'use_goal_publisher',
            default_value='true',
            description='Automatically send CSV goals after Nav2 starts.',
        ),

        # ── Stage simulation ─────────────────────────────────────────────
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{
                'world_file': world_file,
                'one_tf_tree': True,
                'enforce_prefixes': False,
                'use_static_transformations': True,
                'use_sim_time': True,
            }],
            remappings=[
                ('base_scan', 'scan'),
                ('/base_scan', '/scan'),
            ],
            output='screen',
        ),

        # ── Nav2 bringup (AMCL + planner + controller + bt_navigator …) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map':         map_yaml,
                'params_file': nav2_params,
                'use_sim_time': 'true',
                'autostart':    'true',
            }.items(),
        ),

        # ── Goal publisher ───────────────────────────────────────────────
        Node(
            package='second_project',
            executable='goal_publisher',
            name='goal_publisher',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'map',
                'action_name': 'navigate_to_pose',
            }],
            condition=IfCondition(use_goal_publisher),
            output='screen',
        ),

        # ── RViz2 ────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
