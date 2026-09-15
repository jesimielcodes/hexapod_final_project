import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_dynamic_world():
    """Generates a mathematically solvable maze using Depth-First Search."""
    grid_size = 11
    maze = [[1] * grid_size for _ in range(grid_size)]

    def get_unvisited_neighbors(x, y):
        neighbors = []
        directions = [(0, -2), (0, 2), (-2, 0), (2, 0)]
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 < nx < grid_size and 0 < ny < grid_size and maze[ny][nx] == 1:
                neighbors.append((nx, ny, dx, dy))
        return neighbors

    start_x, start_y = 1, 1
    maze[start_y][start_x] = 0
    stack = [(start_x, start_y)]

    while stack:
        current_x, current_y = stack[-1]
        neighbors = get_unvisited_neighbors(current_x, current_y)
        if neighbors:
            nx, ny, dx, dy = random.choice(neighbors)
            maze[current_y + dy // 2][current_x + dx // 2] = 0
            maze[ny][nx] = 0
            stack.append((nx, ny))
        else:
            stack.pop()

    maze[1][2] = 0
    maze[2][1] = 0

    world_xml = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
        <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    wall_thickness = 0.1
    path_width     = 1.5
    wall_height    = 0.5
    wall_counter   = 0

    def get_center_and_size(idx):
        num_paths = idx // 2
        num_walls = idx - num_paths
        start_pos = (num_paths * path_width) + (num_walls * wall_thickness)
        size      = wall_thickness if (idx % 2 == 0) else path_width
        center    = start_pos + (size / 2.0)
        return center, size

    for y in range(grid_size):
        for x in range(grid_size):
            if maze[y][x] == 1:
                pos_x, size_x = get_center_and_size(x)
                pos_y, size_y = get_center_and_size(y)
                world_xml += f"""
    <model name="wall_{wall_counter}">
      <static>true</static>
      <pose>{pos_x} {pos_y} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size_x} {size_y} {wall_height}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size_x} {size_y} {wall_height}</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
                wall_counter += 1

    world_xml += "  </world>\n</sdf>"

    path = '/tmp/dynamic_maze.world'
    with open(path, 'w') as f:
        f.write(world_xml)
    return path


def generate_launch_description():
    pkg_share       = get_package_share_directory('hexapod_description')
    ros_share_dir   = os.path.normpath(os.path.join(pkg_share, '..'))

    # making our stl files findable by Gazebo
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + ros_share_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = ros_share_dir

    dynamic_world_file = generate_dynamic_world()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', f'-r -v4 --render-engine ogre2 --physics-engine gz-physics-bullet-featherstone-plugin {dynamic_world_file}') 
            #specified render engine since we get collision warning messages with DARTsim
        ]
    )

    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        rviz,
    ])