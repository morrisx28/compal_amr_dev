import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xsens_dir = LaunchConfiguration(
            'xsens_dir',
            default=os.path.join(
                get_package_share_directory('xsens_mti_ros2_driver'), 'launch'))
    
    velodyne_dir = LaunchConfiguration(
            'velodyne_dir',
            default=os.path.join(
                get_package_share_directory('velodyne'), 'launch'))
    
    compal_amr_description_dir = LaunchConfiguration(
            'compal_amr_description_dir',
            default=os.path.join(
                get_package_share_directory('compal_amr_description'), 'launch'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([xsens_dir, '/xsens_mti_node.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([velodyne_dir, '/velodyne-all-nodes-VLP16-launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([compal_amr_description_dir, '/compal_amr_state.launch.py']),
        ),
    ])
