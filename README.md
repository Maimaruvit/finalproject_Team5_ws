# finalproject_Team5_ws
ROS2 package for final project for Robotics II MANE 6963

To run the launch file, run the command: ros2 launch multirobotctrl robot_launch.py namespace:="Desired Robot Name" domain_id:="Desired Domain Number"

If the robot has the namespace "robot1", it runs the code to use the lidar and track the closest target. Otherwise, it just runs the yahboomcar_bringup.
