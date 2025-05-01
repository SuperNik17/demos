# 🐢 ROS 2 TurtleBot Follower Experiments

This project contains a set of Python scripts for ROS 2 that enable a **follower** robot to track a **leader** robot, both in real-world experiments and in Gazebo simulation. Obstacle avoidance and advanced control strategies (PID, Sliding Mode) are also included.

---

## 🔽 Clone this Repository

To get started, clone the project from GitHub:

```bash
git clone --branch SuperNik17-patch-1 https://github.com/SuperNik17/demos.git
cd demos/aruco-leader-follower-nav/src/
```

---

## 📦 Dependencies

Before running the nodes, make sure you have the following installed:

- **ROS 2 Humble** (or compatible version)
- **Gazebo Ignition** (recommended: Fortress or later)
- **NVIDIA GPU drivers** (for smooth simulation in Gazebo)
- The following ROS 2 packages must be available in your workspace:

```bash
sudo apt update
sudo apt install   ros-humble-turtlebot4*   ros-humble-image-transport   ros-humble-cv-bridge   python3-opencv   python3-colcon-common-extensions
```

Also, make sure your custom package `turtlebot_pkg` is correctly built inside your <ros2_workspace>:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 📁 Initial Setup

Make sure you are inside your ROS 2 workspace and source the environment:

```bash
cd <ros2_workspace>
source install/setup.bash
```

---

## 🔬 Real-World Following

1. **Publish the pose of the ArUco marker** (leader):
```bash
ros2 run turtlebot_pkg aruco_nav_node
```

2. In a new terminal, run one of the following nodes for the **follower**:

- With PID control:
```bash
ros2 run turtlebot_pkg aruco_nav_node2_PID
```

- With Sliding Mode control:
```bash
ros2 run turtlebot_pkg aruco_nav_node2_SMC
```

3. To move the leader autonomously with a trapezoidal velocity profile:
```bash
ros2 run turtlebot_pkg move_forward
```

---

## 🧱 Real-World Obstacle Avoidance (Experimental)

1. Launch the ArUco node to get the leader pose:
```bash
ros2 run turtlebot_pkg aruco_nav_node
```

2. Then:

- To enable follower path planning with obstacle avoidance:
```bash
ros2 run turtlebot_pkg move_tes_robo_ostacolo_real
```

- To allow the leader to bypass obstacles:
```bash
ros2 run turtlebot_pkg move_doc_rob_intorno_ostacolo
```

---

## 🧪 Gazebo Simulation – Following

1. Launch the two TurtleBots in Gazebo:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py namespace:=doc_robolab
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tes_robolab x:=0.0 y:=-1.0 yaw:=0
```

> 🔧 Make sure Gazebo is running correctly (use the NVIDIA GPU if needed — if CPU usage stays stuck at 30%, there’s a problem).

2. Then, from <ros2_workspace>:

- Node to publish the follower’s pose:
```bash
ros2 run turtlebot_pkg gazebo_sim
```

- Node to publish the leader’s pose (new terminal):
```bash
ros2 run turtlebot_pkg gazebo_sim2
```

- Node to control the following behavior (new terminal):
```bash
ros2 run turtlebot_pkg gazebo_sim3
```

---

## 🧱 Gazebo Simulation – Obstacle Avoidance

1. Launch the robots in Gazebo as shown above.

2. Then:

- Follower pose publisher:
```bash
ros2 run turtlebot_pkg gazebo_sim
```

- Leader pose publisher:
```bash
ros2 run turtlebot_pkg gazebo_sim2
```

- Follower with path planning and obstacle avoidance:
```bash
ros2 run turtlebot_pkg move_tes_robo_ostacolo_gazebo
```

- Leader bypassing the obstacle:
```bash
ros2 run turtlebot_pkg move_doc_rob_intorno_ostacolo
```

---

## ℹ️ Important Notes

- Always `source install/setup.bash` in each terminal.
- Run each node in a separate terminal.
- The main package used is `turtlebot_pkg`.

---

## ✅ Requirements Summary

- ROS 2 Humble
- Gazebo Ignition
- turtlebot4_ignition_bringup package
- Custom package: `turtlebot_pkg`

---

## 👨‍🔬 Credits

Developed at the **Robotics Lab** — University of Armando Nicolella and Pasquale Stingo
