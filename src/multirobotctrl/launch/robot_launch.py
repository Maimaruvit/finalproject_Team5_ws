from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    domain_id = LaunchConfiguration('domain_id')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_id',
            default_value='10',
            description='ROS 2 Domain ID for the robot'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='robot1',
            description='Namespace for the robot'
        ),

        SetEnvironmentVariable(
            'ROS_DOMAIN_ID',
            domain_id
        ),

        # Always run the main Mcnamu driver
        ExecuteProcess(
            cmd=['ros2', 'run', 'yahboomcar_bringup', 'Mcnamu_driver_X3'],
            output='screen'
        ),

        # Only run the Lidar launch file if namespace is 'robot1'
        ExecuteProcess(
            cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_launch.py'],
            output='screen',
            condition=IfCondition([LaunchConfiguration('namespace'), ' == "robot1"'])
        ),

        # Only run the laser tracker if namespace is 'robot1'
        ExecuteProcess(
            cmd=['ros2', 'run', 'yahboomcar_laser', 'laser_Tracker_a1_X3'],
            output='screen',
            condition=IfCondition([LaunchConfiguration('namespace'), ' == "robot1"'])
        )
    ])

