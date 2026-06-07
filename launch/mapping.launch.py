import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    slam_params  = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    rviz_config  = os.path.join(pkg_dir, 'config', 'mapping_rviz.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from rosbag or simulation during mapping',
        ),

        # ── PointCloud2 → LaserScan ──────────────────────────────────────
        # Converts /ugv/rslidar_points (PointCloud2) to /scan (LaserScan)
        # Adjust target_frame to match the lidar TF frame in the bag
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/ugv/rslidar_points'),
                ('scan',     '/scan'),
            ],
            parameters=[{
                'target_frame':        'rslidar',   # adjust if needed
                'transform_tolerance': 0.01,
                'min_height':         -0.4,
                'max_height':          1.0,
                'angle_min':          -3.14159265,
                'angle_max':           3.14159265,
                'angle_increment':     0.00872665,  # ~0.5 deg
                'scan_time':           0.1,
                'range_min':           0.1,
                'range_max':           50.0,
                'use_inf':             True,
                'use_sim_time':        use_sim_time,
            }],
        ),

        # ── SLAM Toolbox (async mapping) ─────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── RViz2 ────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
