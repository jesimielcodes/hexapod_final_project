## Introduction to AI Robotics (CS 353) - Final Project 

   **Hexapod Maze Solver**

  <img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/32011026-a9fd-46de-a910-76484940a530" />
  <img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/1007c161-eceb-4ff2-9ca9-36f77ca1aa7d" />

 

<img width="1550" height="907" alt="WhatsApp Image 2026-04-22 at 22 04 03" src="https://github.com/user-attachments/assets/147d1c5d-2093-40cf-93fa-4213c4b33fbc" />
<img width="1600" height="1136" alt="WhatsApp Image 2026-04-22 at 22 03 35" src="https://github.com/user-attachments/assets/1ccb5aa8-556f-4478-99d7-b34ef5e6b121" />

*More media at the bottom*
---
---
---
---
**READ ME**

This repository contains a fully integrated ROS2 ecosystem for a six-legged maze-solving autonomous hexapod robot. The robot is designed in Onshape, exported as URDF, converted to XACRO files, simulated in Gazebo Sim, and controlled via a python nodes using the ros2_control framework.


# System Architecture
The project is divided into modular ROS2 packages:

- **hexapod_control**: Gait planner and wall following navigation algorithm. Subscribes to '/scan' and publishes to 'hexapod/motion_cmd'.

- **hexapod_bringup**: Top-level launch files to initialize the entire system with a single command. 

- **hexapod_description**: Main Robot URDF/XACRO models (Onshape Import), sensor definitions (LiDAR & Camera), and physical properties.

- **hexapod_mazegenerator**: Generates a new unknown maze each time code is launched.

- **hexapod_slam**: For activating the slam.


---
# Prerequisites

   **ROS2 Version**: Jazzy Jalico

   **Simulator**: Gazebo Sim (Ignition)

   **Dependencies**:

     ```bash
      sudo apt install ros-jazzy-ros2-control \
                       ros-jazzy-ros2-controllers \
                       ros-jazzy-gz-ros2-control
                       ros-jazzy-nav2-lifecycle-manager

    
---

# Installation
- Clone this repository into your workspace src folder:

        ```bash
        cd ~/ros2_ws/src
        git clone <repository-link>
       
- Build the workspace:

      ```bash
      cd ~/ros2_ws
      colcon build --symlink-install
      source install/setup.bash
    


# Running the Simulation

- Bring up the full system (Gazebo, RViz, controllers, and Python nodes) with:

      ```bash
      ros2 launch hexapod_bringup bringup.launch.py
      bash```

  


#  Robot Control & Logic
Hexy primarily navigates using A* path planning on locally generated maps from SLAM, creating intermediate checkpoints that guide it toward the final destination. When the robot encounters uncertainty or gets trapped in complex regions, it switches to a wall-following strategy as a fallback, allowing it to safely navigate along boundaries until a viable path is found, after which it resumes A*-based navigation 

- Perception: The robot subscribes to the /scan topic.
  
- Locomotion Control: Inverse kinematics computes joint angles for stable gait execution.

- Path Planning: Uses SLAM to build local maps and A* to generate subgoal-based paths toward the destination.

- Autonomous Navigation: Hexapod uses A* star algorithm + SLAM to move through unknown maze.
  
- Fallback Strategy: Switches to wall-following when planning fails, then resumes A* navigation once a clear path is available.


# Repository Structure

# Demo Video

<img width="240" height="270" alt="0422(2)" src="https://github.com/user-attachments/assets/fa45515f-2508-4728-a0f9-37095e6af6b1" />


# Deliverables Status

[x] Various ROS2 Packages

[x] URDF/XACRO Modelling (Onshape import)

[x] Gazebo & RViz Integration

[x] Joint Trajectory Controller & Inverse Kinematics for movement

[x] Dynamic Maze Generation

[x] SLAM Occupancy Grid Generation

[x] A* Maze solving algorithm + Wall following

[x] One-Command Bringup Launch File

[x] README file 

[x] Github Best Practices

[x] Demo video

# Challenges

- Reducing maze solving time
  
- Robot seemingly gets stuck at random corners, oscillating back and forth at the same general area for a long while.

  
# Possible Future Works
- Working on current challenges
- Implementing a depth camera in addition to lidar for depth perception.
- Hardware Implementation

More media
 <img width="1600" height="1136" alt="hexapod" src="https://github.com/user-attachments/assets/1453531d-43cd-434d-be04-7f855392f2d6" />

  <img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/5634a6c5-267c-43bf-bcfb-b43817db7784" />

  <img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/adfa8e0e-d42c-440c-b1de-41603762150b" />
