# 🧠 Single Agent Trajectory Control in ROS 2

This ROS 2 node implements a full-state feedback controller (LTI-based) for a differential-drive mobile robot. The robot follows a smooth time-varying reference trajectory in the shape of a rotated lemniscate (∞-loop), generated internally and tracked using feedforward and feedback terms.

Designed for academic experimentation and research in mobile robotics, this node integrates control theory with trajectory generation, publishing control commands and tracking error for analysis.

---

## 🚀 Features

- ⭕️ **Rotated Infinity Trajectory**: Smooth continuous trajectory with tunable parameters.
- 🧮 **Full-State Feedback Control**: Matrix-defined LTI controller using discrete-time formulation.
- 📡 **Odometry-Based Feedback**: Relies on the `/odom` topic for localization.
- 📤 **Reference Broadcasting**: Reference trajectory published to `/reference` for logging.
- 🧪 **Control Data Logging**: Tracking errors and control states published to `/controller`.

---

## 🧰 Dependencies

Make sure the following packages are available in your ROS 2 workspace:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `tf2_ros`
- `action_tutorials_interfaces` (for the `Trajectory` action)

---

## ▶️ Usage


`ros2 run your_package_name central_single_control_node.py`

## 👨‍💻 Authors & Contributors

This project has been developed at the **Universitat Politècnica de València (UPV)** 🇪🇸 and **Departamento de Ingeniería de Sistemas y Automática (DISA)** as part of ongoing research in mobile robotics and control of differential-drive agents. It combines theory, simulation, and real-world experimentation. <br>
Special thanks to **Ricardo Núñez Saez** and **Prof. Leopoldo Armesto**.
