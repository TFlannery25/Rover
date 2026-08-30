from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('my_rover_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_rover.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'my_rover_bridge.yaml')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch', 'gz_sim.launch.py')

    return LaunchDescription([

        # Empty Gazebo world, running (-r) rather than paused
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': 'empty.sdf -r'}.items()
        ),

        # Publishes /robot_description and the TF tree, same as the display-only launch
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawns the robot INTO the running Gazebo world, reading its
        # description from the /robot_description topic above
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'my_rover',
                '-z', '0.1',
            ],
            output='screen',
        ),

        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
    ])