import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rover_config_dir = os.path.expanduser(
        '~/rover_ws/src/my_rover_nodes/config')
    ekf_params = os.path.join(rover_config_dir, 'ekf_params.yaml')
    nav2_params = os.path.join(rover_config_dir, 'nav2_params.yaml')

    gazebo_launch = os.path.join(
        get_package_share_directory('my_rover_description'),
        'launch', 'gazebo.launch.py')

    # Turtle bot launch stuff, for reference. We don't use it because we want to launch our own gazebo world and robot description.
    #gazebo_launch = os.path.join(
    #    get_package_share_directory('turtlebot3_gazebo'),
    #   'launch', 'turtlebot3_world.launch.py')

    nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch', 'navigation2.launch.py')

    explore_launch = os.path.join(
        get_package_share_directory('explore_lite'),
        'launch', 'explore.launch.py')

    mode = LaunchConfiguration('mode')
    is_autonomous = IfCondition(
        # True only when mode == 'autonomous'
        launch.substitutions.PythonExpression(["'", mode, "' == 'autonomous'"])
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'mode',
            default_value='autonomous',
            description="Operating mode for this session: 'autonomous' or 'manual'. Chosen once at launch, not switchable at runtime."
        ),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # ekf_node always runs, regardless of mode
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_params, {'use_sim_time': True}],
                )
            ]
        ),

        # nav2 (with bundled slam_toolbox) — ONLY in autonomous mode
        TimerAction(
            period=45.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'use_sim_time': 'True',
                        'slam': 'True',
                        'params_file': nav2_params,
                    }.items(),
                    condition=is_autonomous,
                )
            ]
        ),

        # explore_lite — ONLY in autonomous mode
        TimerAction(
            period=60.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(explore_launch),
                    condition=is_autonomous,
                )
            ]
        ),
    ])