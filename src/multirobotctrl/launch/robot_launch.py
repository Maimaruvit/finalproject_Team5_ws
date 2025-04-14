from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    domain_id = LaunchConfiguration('domain_id')
    namespace = LaunchConfiguration('namespace')

    # Declare shared variables for process completion tracking
    form1_done = []
    form2_done = []

    # Declare launch arguments and environment setup
    ld = LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='10'),
        DeclareLaunchArgument('namespace', default_value='robot1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
    ])

    # Initial Processes
    form1 = ExecuteProcess(
        cmd=['ros2', 'run', 'multirobotctrl', 'agreement_algo'],
        output='screen'
    )

    form2 = ExecuteProcess(
        cmd=['ros2', 'run', 'multirobotctrl', 'move_robot'],
        output='screen'
    )

    # Persistent Process (runs entire time)
    persistent = ExecuteProcess(
        cmd=['ros2', 'run', 'yahboomcar_bringup', 'Mcnamu_driver_X3'],
        output='screen'
    )

    ld.add_action(form1)
    ld.add_action(form2)
    ld.add_action(persistent)

    # Function to launch remaining processes after both form1 and form2 are done
    def launch_remaining(context):
        return [
            ExecuteProcess(
                cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_launch.py'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'run', 'yahboomcar_laser', 'laser_Tracker_a1_X3'],
                output='screen',
            )
        ]

    # Handlers for when form1 and form2 exit
    def on_form1_exit(event, context):
        form1_done.append(True)
        if form2_done:
            for action in launch_remaining(context):
                ld.add_action(action)

    def on_form2_exit(event, context):
        form2_done.append(True)
        if form1_done:
            for action in launch_remaining(context):
                ld.add_action(action)

    # Register event handlers
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=form1,
        on_exit=on_form1_exit
    )))

    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=form2,
        on_exit=on_form2_exit
    )))

    return ld

