from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    domain_id = LaunchConfiguration('domain_id')
    namespace = LaunchConfiguration('namespace')

    form1_done = []
    form2_done = []

    form1 = ExecuteProcess(
        cmd=['ros2', 'run', 'multirobotctrl', 'go_to_start_pos'],
        output='screen'
    )

    form2 = ExecuteProcess(
        cmd=['ros2', 'run', 'multirobotctrl', 'coordinate_ctrl'],
        output='screen'
    )

    persistent = ExecuteProcess(
        cmd=['ros2', 'run', 'yahboomcar_bringup', 'Mcnamu_driver_X3'],
        output='screen'
    )

    # Predefine remaining processes
    lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_launch.py'],
        output='screen',
    )
    tracker = ExecuteProcess(
        cmd=['ros2', 'run', 'yahboomcar_laser', 'laser_Tracker_a1_X3'],
        output='screen',
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='10'),
        DeclareLaunchArgument('namespace', default_value='robot1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        form1,
        form2,
        persistent,
    ])

    def try_launch_remaining():
        if form1_done and form2_done:
            ld.add_action(lidar)
            ld.add_action(tracker)

    def on_form1_exit(event, context):
        form1_done.append(True)
        try_launch_remaining()

    def on_form2_exit(event, context):
        form2_done.append(True)
        try_launch_remaining()

    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=form1,
        on_exit=on_form1_exit
    )))

    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=form2,
        on_exit=on_form2_exit
    )))

    return ld

