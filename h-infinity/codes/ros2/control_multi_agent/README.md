#  Centralized Control Node for Multi-Robot Systems (ROS 2)

This repository contains a centralized controller for coordinating the motion of **two differential-drive mobile robots** (e.g., TurtleBot3) using ROS 2. The system supports **real-time trajectory generation**, **state-space control**, and **sensor fusion from multiple localization sources**.

---

##  Overview

This node allows full control over a multi-agent system, enabling behaviors such as:
- Leader-follower navigation
- Formation maintenance
- Relative pose control
- Distributed feedback from AMCL, ArUco, or odometry

Key features:
-  Trajectory generation (sinusoidal or φ-parametrized)
-  Flexible control schemes (`Kp`, `Kpid`, `Kcentralized`) via JSON
-  Multiple localization inputs (AMCL, ArUco, LSM, Simulated Odom)
-  Reference broadcasting for visualization and logging
-  Designed for real-world testing and Gazebo simulation

##  Requirements

- ROS 2 (tested on **Humble** / **Foxy**)
- Python 3.8+
- Dependencies:
  - `geometry_msgs`
  - `nav_msgs`
  - `std_msgs`
  - `std_srvs`
  - `rclpy`

Ensure controller configs are present under:
central_pkg/config/ <br>
  ├── Kp.json <br>
  ├── Kpid.json <br>
  └── Kcentralized.json <br>


  ---

##  Acknowledgements

This work was conducted entirely at the **Universitat Politècnica de València (UPV)** 🏫 as part of ongoing research in multi-robot systems.

I gratefully acknowledge:

- **Prof. Leopoldo Armesto** for his exceptional guidance, technical insight, and unwavering support throughout the project.
- **Ricardo Núñez Saez** for completing the experimental phase and implementing robust **AMCL-based multi-agent localization** on real robots.

  

