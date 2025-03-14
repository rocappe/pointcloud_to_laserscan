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
            package='pointcloud_to_laserscan', executable='cut_pointcloud_node',
            remappings=[('cloud_in', '/zed2/zed_node/point_cloud/cloud_registered'),],
            parameters=[{
                'target_frame': 'zed2_left_camera_frame',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 1.5,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'range_min': 0.20,
                'range_max': 5.0,
                'is_optical':False
            }],
            name='cut_pointcloud_node'
        )
    ])
