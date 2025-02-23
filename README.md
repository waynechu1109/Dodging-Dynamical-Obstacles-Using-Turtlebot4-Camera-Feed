<h3 align="center">
  <a href="https://waynechu1109.github.io/paper/Dodging Dynamical Obstacles.pdf">Paper</a> |
  <a href="https://waynechu1109.github.io/slides/PMAE2023_Presentation.pdf">Slides</a> |
  <a href="https://youtu.be/wdz8laYMFX0?si=9YsBBpm0sXYLn-bW">Demo</a>
  <!-- <a href="https://opendrivelab.com/e2ead/UniAD_plenary_talk_slides.pdf">Slides</a> -->
</h3>

# Dodging Dynamical Obstacles Using Turtlebot4 Camera Feed

## Overview
This project explores a vision-based approach for Turtlebot4 to dynamically dodge obstacles while navigating toward a designated destination. Unlike previous works that rely on expensive sensors like LiDAR or event cameras, this research focuses on using an affordable stereo camera, the OAK-D Pro, for real-time obstacle detection and avoidance. The implementation includes stereo vision processing, RRT* path planning, Bézier curve smoothing, and a unicycle-based control model.

## Features
- **Obstacle Detection:** Uses OAK-D Pro camera to detect obstacles and determine their coordinates.
- **Path Planning:** Implements RRT* (Rapidly-exploring Random Trees) to plan collision-free paths.
- **Path Smoothing:** Uses Bézier curves to create a smooth trajectory for the Turtlebot4.
- **Motion Control:** Applies a unicycle model for velocity and angular velocity calculations.
- **ROS2 Integration:** Built using ROS2, allowing modular and scalable implementation.

## System Requirements
- **Hardware:**
  - Turtlebot4
  - OAK-D Pro Camera
  - Raspberry Pi 4B (or compatible computational unit)
- **Software:**
  - ROS2 (Robot Operating System 2)
  - OpenCV
  - Luxonis DepthAI SDK
  - Python 3

## Installation
### 1. Clone the Repository
```bash
$ git clone https://github.com/waynechu1109/Dodging-Dynamical-Obstacles-Using-Turtlebot4-Camera-Feed.git
$ cd Dodging-Dynamical-Obstacles-Using-Turtlebot4-Camera-Feed
```

### 2. Set Up ROS2 Environment
Ensure that ROS2 is installed and sourced before running the system:
```bash
$ source /opt/ros/humble/setup.bash
```

## Running the Project
### 1. Start the Camera Node
```bash
$ ros2 launch turtlebot4_camera camera.launch.py
```

### 2. Run the Obstacle Detection Module
```bash
$ ros2 run obstacle_detection detection_node
```

### 3. Run the Path Planning and Control Node
```bash
$ ros2 run path_planning planner_node
```

### 4. Execute the Turtlebot4 Movement
```bash
$ ros2 run robot_controller movement_node
```

## Implementation Details
### 1. Obstacle Detection
- The OAK-D Pro camera provides depth information to estimate obstacle positions.
- ROS2 topics `/camera_data` and `/obstacle_info` handle obstacle tracking.

### 2. Transformation to Odometry Frame
- Converts obstacle coordinates from the camera frame to the robot's odometry frame using transformation matrices.

### 3. Path Planning with RRT*
- Generates a feasible path avoiding obstacles detected in real-time.

### 4. Path Smoothing with Bézier Curves
- Smooths the RRT* path to create a natural, drivable trajectory.

### 5. Motion Control
- Implements a unicycle-based control model to compute velocity and angular velocity.

## Results
### **Simulation**
The robot successfully detects moving obstacles, recalculates paths, and dodges them dynamically while progressing toward its destination.

## Future Applications
- **Self-Driving Cars:** Potential to reduce costs by replacing LiDAR-based obstacle avoidance with camera-based systems.
- **Unmanned Aerial Vehicles (UAVs):** Enhancing drone navigation safety by incorporating vision-based obstacle avoidance.

## Contact
Developed by **Wei-Teng Chu** under the mentorship of **Neilabh Banzal, Parth Paritosh, Scott Addams**, and **Prof. Jorge Cortés** at the **Multi-Agent Robotics (MURO) Lab, UC San Diego**.

For inquiries, feel free to reach out via GitHub Issues or email.
