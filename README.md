# Assignment 4: Build and Simulate a Differential Drive Robot with LiDAR
## Overview
This assignments implements a complete mobile robot simulation from scratch using URDF, TF, Plugins, RViz, and Gazebo (`gz sim`).
The robot is a three-wheel differential drive platform, simulated with a Gazebo `DiffDrive` plugin, publishing odometry on `/odom`, broadcasting a full TF tree, and carrying a LiDAR sensor that publishes `/scan`.
This assignment covers the complete ROS 2 robot simulation workflow: building a robot from URDF, visualizing it in RViz, simulating it in Gazebo, controlling it via keyboard teleoperation, and adding LiDAR sensing.

---
## Objectives
-
-

---
## Project Structure
```
my_robot_description/
├── config/
│   └── gz_bridge.yaml
│
├── launch/
│   └── gazebo.launch.py
│
├── urdf/
│   └── robot.urdf.xacro
│   └── robot.gazebo.xacro
│
├── meshes/
│   └── lidar.STL
│
├── rviz/
│   └── robot.rviz
│
├── package.xml
└── CMakeLists.txt
```
---
### Robot Description
Links:
Link	Geometry	Description
`base_footprint`	Box	Main Block
`base_link`	Box	Main chassis
`left_front_wheel_link`	Cylinder	Front-left wheel (driven)
`right_front_wheel_link`	Cylinder	Front-right wheel (driven)
`caster_wheel_link`	Sphere	Rear (Non-Driven)
`lidar_link`	Mesh (STL)	LiDAR sensor, mounted on top of chassis

Joints:
Joint	Type	Parent → Child
`footprint_joint`	`fixed`	`base_footprint` → `base_link`
`*_wheel_joint` (×2) 	`base_link` → `*_wheel_link`
` caster_joint` (×2)	`base_link` → `caster_wheel_link`
`lidar_joint`	`fixed`	`base_link` → `lidar_link`

---
### Gazebo Plugins
Plugin	Purpose
`gz::sim::systems::DiffDrive`	Subscribes to `/cmd_vel`, drives the front and rear axles, publishes `/odom` and the `odom → base_link` TF
`gz::sim::systems::JointStatePublisher`	Publishes wheel joint angles for `robot_state_publisher` to broadcast wheel TF
`gz::sim::systems::OdometryPublisher`	Publishes odometry data
`gpu_lidar` sensor (on `lidar_link`)	Simulates the LiDAR, publishes scan data on `/scan`

---
### Topics Used
Topic	Type	Direction	Purpose
`/cmd_vel`	`geometry_msgs/msg/Twist`	ROS 2 → Gazebo	Robot velocity commands
`/odom`	`nav_msgs/msg/Odometry`	Gazebo → ROS 2	Robot pose/velocity estimate
`/tf`, `/tf_static`	`tf2_msgs/msg/TFMessage`	Gazebo → ROS 2	Transform tree
`/joint_states`	`sensor_msgs/msg/JointState`	Gazebo → ROS 2	Wheel joint angles (needed for wheel TF)
`/scan`	`sensor_msgs/msg/LaserScan`	Gazebo → ROS 2	LiDAR scan data
`/clock`	`rosgraph_msgs/msg/Clock`	Gazebo → ROS 2	Simulation time, for `use_sim_time`
All of the above are connected via a `ros_gz_bridge` `parameter_bridge` node defined inline in `gazebo.launch.py`.

---
### Requirements
ROS 2 Jazzy
Gazebo Sim (`gz sim`, Harmonic) via `ros_gz_sim` / `ros_gz_bridge`
`robot_state_publisher`, `joint_state_publisher`
`turtlebot3_gazebo` (for the `turtlebot3_world` world file)
`teleop_twist_keyboard`
`rviz2`

---
## Build
```bash
cd ~/ros2_ws
colcon build --packages-select ros2-robot
source install/setup.bash
```
---
### How to Run

---
### Verification
The system was tested using a custom 3-wheel differential drive robot in `gz sim` (Gazebo Harmonic), spawned inside `turtlebot3_world`. The following was verified:
Robot URDF loads with no XML errors, no floating links
Robot displays correctly in RViz with a fully connected TF tree
Robot spawns successfully in Gazebo, stable and above the ground plane
`DiffDrive` plugin correctly subscribes to `/cmd_vel` and drives both axles
Robot moves forward, backward, and rotates left/right via teleop, with wheels rotating correctly
`/odom` updates continuously with correct position and orientation while driving
LiDAR link, mesh reference, and `gpu_lidar` sensor plugin are correctly defined; `/scan` is published on the Gazebo side and successfully bridged to ROS 2
Final TF tree is fully connected: `odom → base_footprint → base_link → {lidar_link, 2× wheel links}`, with no disconnected frames

---
URDF View (VS Code URDF Visualizer):

<img width="430" height="246" alt="Screenshot 2026-07-01 150744" src="https://github.com/user-attachments/assets/9816bc39-f7aa-4fba-9d6b-5c6e810009a2" />


RViz — Robot Model & TF:

<img width="762" height="518" alt="Screenshot 2026-06-23 123716" src="https://github.com/user-attachments/assets/619ede01-0107-4ed0-ab87-497d0a79c536" />

Gazebo Simulation (robot spawned in turtlebot3_world):

<img width="669" height="378" alt="Screenshot 2026-06-22 171932" src="https://github.com/user-attachments/assets/dd15539c-df5f-459b-a637-2837cb81580e" />

Teleop Terminal:

<img width="937" height="507" alt="Screenshot 2026-06-22 173122" src="https://github.com/user-attachments/assets/c91ef0c4-1e7f-4b8c-89fe-ebf85c4dee88" />

RViz — Odometry (Fixed Frame = odom):

<img width="1061" height="672" alt="image" src="https://github.com/user-attachments/assets/a4e85e17-de21-409f-a0df-c7d13bf1f024" />

URDF View (With Lidar "Cylinder")

<img width="214" height="175" alt="image" src="https://github.com/user-attachments/assets/73379417-0649-455e-8eea-93b1e6eebd47" />

TF Tree (`ros2 run tf2_tools view_frames`):

<img width="953" height="336" alt="image" src="https://github.com/user-attachments/assets/3fe9a284-0371-4291-a972-213e9566c56d" />

Lidar Scanning: 

<img width="915" height="665" alt="image" src="https://github.com/user-attachments/assets/aa4d4f2a-e2d8-4224-a160-1c81c9b65633" />


Demo Video Link


https://github.com/user-attachments/assets/8d5c92e4-7e56-4d62-8125-c6f9988aa91f



---
