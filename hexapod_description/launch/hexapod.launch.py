import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share  = get_package_share_directory('hexapod_description')
    maze_share = get_package_share_directory('hexapod_mazegenerator')

    # --- Robot Description ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # --- Spawn coordinates from config ---
    config_path = os.path.join(maze_share, 'config', 'spawn.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    spawn_x   = str(config['hexapod_spawn']['x'])
    spawn_y   = str(config['hexapod_spawn']['y'])
    spawn_z   = str(config['hexapod_spawn']['z'])
    spawn_yaw = str(config['hexapod_spawn']['yaw'])

    # --- Nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'hexapod',
            '-x',     spawn_x,
            '-y',     spawn_y,
            '-z',     spawn_z,
            '-Y',     spawn_yaw,
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Delaying spawn 2s to let robot_state_publisher publish /robot_description first
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_entity])

    return LaunchDescription([
        robot_state_publisher,
        delayed_spawn,
    ])