import os
import launch
import launch_ros.actions

def generate_launch_description():
    package_path = '/home/ibrahim/ros2_ws/src/mobile_robot'  # Replace this with your actual package path
    urdf_file_path = os.path.join(package_path, 'urdf', 'my_robot.xacro')  # Path to your URDF file
    rviz_config_path = os.path.join(package_path, 'config', 'config.rviz')  # Path to your RViz config file

    return launch.LaunchDescription([
        # Load URDF file
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path, 'r').read()}]
        ),
        launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output = 'screen',
        ),
        # Launch RViz2 with config file
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
