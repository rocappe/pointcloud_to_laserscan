from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='camera',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                'transform_tolerance': 0.01,
                'target_frame': 'base_link',
                'tf_sync': False,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -0.7592,  # -87°/2
                'angle_max': 0.7592,  # 87°//2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.0667,
                'range_min': 0.30,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
