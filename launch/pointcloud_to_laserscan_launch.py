from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/zed2/zed_node/point_cloud/cloud_registered'),
                        ('scan', '/new_scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.0,
                # 'angle_min': -3.14,  
                # 'angle_max': 3.14,  
                # 'angle_increment': 0.017453,  # M_PI/360.0
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.03333,
                'range_min': 0.25,
                'range_max': 9.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'is_optical':True
            }],
            name='pointcloud_to_laserscan'
        )
    ])
    #/camera/depth/color/points
    #camera_depth_frame
