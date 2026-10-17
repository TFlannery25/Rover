from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/stm32'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        Node(
            package='my_rover_nodes',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': False,
            }],
        ),
    ])