import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    controller_pkg_path = get_package_share_directory('hexapod_control')

    controller_yaml = os.path.join(
        controller_pkg_path, 'config', 'hexapod_controller.yaml'
    )

    # Spawn controllers — delay slightly to let controller_manager come up
    spawn_controllers = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'hexapod_controller',
                    '--controller-manager', '/controller_manager',
                    '--activate',
                ],
                output='screen',
            ),
        ]
    )

    # --- Walker pkg nodes ---
    gait_planner = Node(
        package='hexapod_control',
        executable='gait_planner',
        name='gait_planner',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    wall_follower = Node(
        package='hexapod_control',
        executable='wall_follower',
        name='wall_follower',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.15,
            'angular_speed': 0.5,
            'wall_follow_distance': 0.4,
            'waypoint_threshold': 0.25,
        }],
        output='screen',
    )

    astar_planner = Node(
        package='hexapod_control',
        executable='local_astar_navigator',
        name='local_astar_navigator',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen',
    )

    odom_tfbroadcaster = Node(
        package='hexapod_control',
        executable='odom_tfbroadcaster',
        name='odom_tfbroadcaster',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        spawn_controllers,
        gait_planner,
        # wall_follower, # wall follower has been included in the astar_planner
        astar_planner,
        odom_tfbroadcaster,
    ])