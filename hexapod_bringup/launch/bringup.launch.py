import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    maze_pkg        = get_package_share_directory('hexapod_mazegenerator')
    description_pkg = get_package_share_directory('hexapod_description')
    hexapod_control      = get_package_share_directory('hexapod_control')
    slam_pkg        = get_package_share_directory('hexapod_slam')

    # Gazebo world + bridge + RViz  (t=0s)
    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(maze_pkg, 'launch', 'maze.launch.py')
        )
    )

    # Robot state publisher + spawn  (t=7s, after Gazebo is up)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'hexapod.launch.py')
        )
    )
    delayed_robot = TimerAction(period=10.0, actions=[launch_robot])

    # Controllers + walker nodes  (t=12s, after robot is spawned and controller_manager is ready)
    launch_walker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hexapod_control, 'launch', 'hexy_walker.launch.py')
        )
    )
    delayed_walker = TimerAction(period=15.0, actions=[launch_walker])

    #  SLAM  (t=18s, after robot and controllers are running)
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async.launch.py')
        )
    )
    delayed_slam = TimerAction(period=25.0, actions=[launch_slam])

    return LaunchDescription([
        launch_world,
        delayed_robot,
        delayed_walker,
        delayed_slam,
    ])