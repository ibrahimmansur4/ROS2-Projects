import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ignition_robot')
    
    # Path to the custom world file
    world_file_name = 'empty_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path], 
        'on_exit_shutdown': 'true'}.items()
        # launch_arguments={'world': world_path}.items(),
    )


    # URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'ignbot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'ignbot.rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'diffbot',
                  '-z', '0.1'],
        output='screen'
    )
	
    # Bridge
    bridge = Node(
	package='ros_gz_bridge',
	executable='parameter_bridge',
	arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
		   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
		   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
	           '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
	           '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
	           '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
	output='screen'
    )
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        bridge,
        rviz,
    ])