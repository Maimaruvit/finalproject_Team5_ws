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

    persistent1 = ExecuteProcess(
        cmd=['ros2', 'run', 'yahboomcar_bringup', 'Mcnamu_driver_X3'],
        output='screen'
    )

    persistent2 = ExecuteProcess(
        cmd=['ros2', 'run', 'yahboomcar_laser', 'laser_Tracker_a1_X3'],
        output='screen',
    )

    # Predefine remaining processes
    lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_launch.py'],
        output='screen',
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(...),
        SetEnvironmentVariable(...),
        form1,  # Initial position setup
        persistent1,  # Long-running motor driver
        persistent2, # Long-running lidar "driver"
    ])

    # Chain: form1 -> form2 -> sensors
    def start_form2_after_form1(event, context):
        return [form2]  # Launch form2 after form1 exits

    def start_sensors_after_form2(event, context):
        return [lidar]  # Launch sensors after form2 exits

    # Register event handlers
    ld.add_action(RegisterEventHandler(OnProcessExit(
            target_action=form1,
            on_exit=start_form2_after_form1
    )))

    ld.add_action(RegisterEventHandler(OnProcessExit(
            target_action=form2,
            on_exit=start_sensors_after_form2
    )))

    return ld
