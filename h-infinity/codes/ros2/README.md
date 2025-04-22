# 🔧 ROS 2 Serial Pose Publisher (EMA & SMA)

Author: **Armando Nicolella**  
Status: 🛰️ Ready for MarvelMind SuperBeacon integration  
ROS 2 Compatibility: ✅ Foxy / Humble / Iron

---

## 📦 Overview

This repository contains two ROS 2 Python nodes that read positional data (x, y, z) from a serial interface (e.g., Arduino, microcontrollee), apply filtering (EMA or SMA), and publish the data as a `geometry_msgs/Pose` message on the `pose` topic.

These scripts are designed to **bridge low-level serial devices with the ROS 2 ecosystem**, enabling seamless integration of external localization systems like the **MarvelMind SuperBeacon**.



## 🛰️ Use Case: MarvelMind SuperBeacon Localization

These nodes are intended to transform the raw serial output of a MarvelMind SuperBeacon into a ROS 2-friendly format. Once integrated, this allows you to:
- Monitor robot position in RViz
- Feed position into SLAM / Navigation stacks
- Combine with other sensory data for robust localization
- Deploy in multi-robot swarm systems



## 🧠 Available Nodes

### `read_serial_publisher_ema_2.py`
- **Filter:** Exponential Moving Average (EMA)
- **Highlights:**
  - Delays filtering until 10 initial samples are collected
  - Smooth and responsive to sudden signal changes
- **Use case:** Noisy or jittery environments

### `read_serial_publisher_sma.py`
- **Filter:** Simple Moving Average (SMA)
- **Highlights:**
  - Averages the last 10 samples equally
- **Use case:** Smooth signals or slower dynamic environments



## 🔌 Serial Input Format
Each line represents:
- `x` position in meters
- `y` position in meters
- `z` position in meters

🛠️ Default serial config:
- Port: `/dev/ttyUSB1`
- Baudrate: `115200`

## 📤 ROS 2 Output

Each node publishes a `geometry_msgs/msg/Pose` message on the `pose` topic:

```bash
ros2 topic echo /pose

