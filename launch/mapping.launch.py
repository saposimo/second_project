import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_slam_node(context, *args, **kwargs):
    slam_mode = LaunchConfiguration('slam_mode').perform(context).lower()
    if slam_mode == 'sync':
        executable = 'sync_slam_toolbox_node'
    elif slam_mode == 'async':
        executable = 'async_slam_toolbox_node'
    else:
        raise RuntimeError("slam_mode must be either 'async' or 'sync'")

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mode': 'mapping',
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                },
            ],
        )
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    default_slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    default_rviz = os.path.join(pkg_dir, 'config', 'mapping_rviz.rviz')

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan', '/scan_raw'),
        ],
        parameters=[{
            'target_frame': 'rslidar',
            'transform_tolerance': 0.01,
            'min_height': -0.4,
            'max_height': 1.0,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    scan_qos_relay = Node(
        package='second_project',
        executable='scan_qos_relay',
        name='scan_qos_relay',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': LaunchConfiguration('scan_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('slam_mode', default_value='async', description='Use async or sync SLAM Toolbox mapping.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use /clock from rosbag playback.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Reliable LaserScan topic consumed by SLAM Toolbox.'),
        DeclareLaunchArgument('odom_frame', default_value='UGV_odom', description='Odometry frame from the bag.'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame published by SLAM Toolbox.'),
        DeclareLaunchArgument('base_frame', default_value='UGV_base_link', description='Robot base frame from the bag.'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params, description='SLAM Toolbox params file.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config file.'),
        pointcloud_to_laserscan,
        scan_qos_relay,
        OpaqueFunction(function=make_slam_node),
        rviz,
    ])
