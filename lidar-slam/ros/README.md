# Autonomous Navigation Setup for AgileX Scout Mini

This repository contains the setup to enable **autonomous navigation** for the **AgileX Scout Mini** robot using a **Velodyne VLP-16** LiDAR and a **NVIDIA Jetson AGX Xavier** (Ubuntu 18.04 + ROS Noetic).

> **Warning**: The linked packages are the originals. The ones used here may have been modified. Always refer to the versions included in this repository.

---

## Table of Contents
- [Requirements](#requirements)
- [Installation and Setup](#installation-and-setup)
  - [1. ugv_sdk - CAN Interface Setup](#1-ugv_sdk---can-interface-setup)
  - [2. Robot Model - scout_ros](#2-robot-model---scout_ros)
  - [3. Velodyne LiDAR Driver](#3-velodyne-lidar-driver)
  - [4. PointCloud to LaserScan Conversion](#4-pointcloud-to-laserscan-conversion)
  - [5. Direct Lidar Odometry (DLO)](#5-direct-lidar-odometry-dlo)
  - [6. Mapping - Gmapping](#6-mapping---gmapping)
  - [7. (Optional) Localization - AMCL](#7-optional-localization---amcl)
  - [8. Autonomous Navigation - move_base](#8-autonomous-navigation---move_base)
- [Quick Launch](#quick-launch)
- [Saving a Map for AMCL Usage](#saving-a-map-for-amcl-usage)
- [Multi-Agent Experience](#multi-agent-experience)
- [Notes and Recommendations](#notes-and-recommendations)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Requirements
- **Robot**: AgileX Scout Mini
- **Sensors**: Velodyne VLP-16
- **Computer**: NVIDIA Jetson AGX Xavier
- **OS**: Ubuntu 18.04
- **Middleware**: ROS Noetic

---

## Installation and Setup

### 1. ugv_sdk - CAN Interface Setup
Repository: [ugv_sdk](https://github.com/westonrobot/ugv_sdk)

- Build the SDK.
- Enable kernel module `gs_usb`.
- Configure CAN interface:
  ```bash
  sudo ip link set can0 up type can bitrate 500000
  ```
- Verify CAN packet transmission:
  ```bash
  candump can0
  ```

### 2. Robot Model - scout_ros
Repository: [scout_ros](https://github.com/agilexrobotics/scout_ros)

Launch robot base:
```bash
roslaunch scout_bringup scout_mini_robot_base.launch
```
This will automatically start:
- `robot_description`
- `joint_state_publisher`
- `robot_state_publisher`

### 3. Velodyne LiDAR Driver
Repository: [velodyne_driver](https://github.com/ros-drivers/velodyne)

Launch the driver:
```bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

> **Important**: Configure the LiDAR network settings (static IP, DHCP off, Host IP, Sensor IP, Gateway IP).  
> If the LiDAR IP is unknown, use tools like **Wireshark** to detect it.

### 4. PointCloud to LaserScan Conversion
Transform point cloud to laser scan:
```bash
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/velodyne_points
```

### 5. Direct Lidar Odometry (DLO)
Repository: [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry)

Launch DLO:
```bash
roslaunch direct_lidar_odometry dlo.launch
```
> **Notes:**
> - Disable IMU usage in the launch file if not needed.
> - The DLO publishes frames under a `/robot` namespace.
> - Ensure rigid and fixed frame relations between `velodyne` and `base_link` in the URDF/Xacro files.

### 6. Mapping - Gmapping
Repository: [gmapping](http://wiki.ros.org/gmapping)

Launch Gmapping:
```bash
roslaunch gmapping gmapping.launch
```
You can either:
- Build maps in real-time.
- Save maps for future use.

### 7. (Optional) Localization - AMCL
Repository: [navigation](https://github.com/ros-planning/navigation)

If using a saved map:
```bash
roslaunch amcl amcl_diff.launch
```
> In our environment, DLO proved sufficient without AMCL.

### 8. Autonomous Navigation - move_base
Launch move_base with Gmapping:
```bash
roslaunch move_base move_base_gmapping.launch
```
Waypoints can be set either via:
- RViz (2D Nav Goal).
- Custom Python scripts (waypoint recording and execution).

Path Planners used:
- `TrajectoryPlannerROS`
- `NavfnROS`

---

## Quick Launch

Two dedicated launch files are provided to automatically start the entire navigation stack:

- **Navigation with AMCL and a pre-built map**:
  ```bash
  roslaunch unina_nav_pkg unina_amcl.launch
  ```

- **Navigation without AMCL (real-time SLAM with Gmapping and DLO)**:
  ```bash
  roslaunch unina_nav_pkg unina_no_amcl.launch
  ```

Simply launch the appropriate file depending on the desired navigation mode.  
No manual node-by-node launch is required.

---

## Saving a Map for AMCL Usage

If you want to use AMCL with a saved map, follow these steps:

1. Launch the robot and sensors:
```bash
roslaunch scout_bringup scout_mini_robot_base.launch
roslaunch velodyne_pointcloud VLP16_points.launch
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/velodyne_points
roslaunch direct_lidar_odometry dlo.launch
```

2. Launch GMapping to build the map:
```bash
roslaunch gmapping gmapping.launch
```

3. Move the robot manually (teleoperation) to explore the environment.

4. Save the generated map:
```bash
rosrun map_server map_saver -f /path/where/you/want/to/save/your_map
```

5. Terminate all nodes.

6. Launch autonomous navigation using AMCL and the saved map:
```bash
roslaunch unina_nav_pkg unina_amcl.launch
```

7. Setting navigation goals:
- Use RViz `2D Nav Goal`, or
- Collect waypoints during teleoperation:
```bash
rosrun unina_nav_pkg collect_waypoints_cart.py
rosrun unina_nav_pkg navigationina.py
```

> **Notes**:  
> - When AMCL is launched, provide an initial pose using `2D Pose Estimate` in RViz for faster convergence.  
> - The best RViz configuration is provided in `unina_config.rviz`.
> - Xavier board can be accessed via SSH or using NoMachine/RustDesk for remote control.

---

## Multi-Agent Experience

The setup has been extended to support multiple robots sharing the same map:

- **Robot Master**:
  - Builds or loads the map.
  - Publishes `/map`.
  - Runs DLO, Gmapping (if needed), and move_base.

- **Robot Slave**:
  - Subscribes to `/map` published by the Master.
  - Runs DLO and move_base (no Gmapping, no map building).

> **Important**:  
> - Namespaces (`robot1/`, `robot2/`, etc.) must be used to separate topics and frames.
> - Each robot publishes its own odometry via DLO.
> - All robots use the same global `/map` frame for navigation and goal setting.

An example launch file structure for multi-agent deployment is provided (`multi_agent.launch`).

---

## Notes and Recommendations
- **Frame Consistency**: Rigidly connect all sensor frames (e.g., Velodyne) to the robot model.
- **Mapping**: Use real-time mapping unless a static environment is guaranteed.
- **Navigation Parameters**: Tune them according to robot dynamics and environment features.
- **Environment Sensitivity**: This setup is tested in structured indoor environments. External environments may require adaptations.

---

## License
This project is for research and development purposes.

---

## Acknowledgements
- AgileX Robotics
- ROS Community
- Vectr-UCLA Direct Lidar Odometry Developers

---

Feel free to open issues or suggestions to improve this setup.

---

> _"Making robots autonomous, one laser scan at a time."_ 🚀
