# 📡 MarvelMind Serial Publisher (Arduino)

This Arduino sketch reads position data from a **MarvelMind SuperBeacon** via serial and sends it over USB in a simple, ROS 2-compatible format.

## 🔧 Description

- Communicates with a MarvelMind beacon via UART
- Converts position data from millimeters to meters
- Sends the X, Y, Z coordinates as a space-separated string over the Serial port:
  

- Output is intended to be parsed and published inside ROS 2 using scripts like [`read_serial_publisher_ema_2.py`](h-infinity/codes/ros2/read_serial_ros2/read_serial_publisher_ema_2.py) or [`read_serial_publisher_sma.py`](h-infinity/codes/ros2/read_serial_ros2/read_serial_publisher_sma.py)

## ⚙️ Hardware Requirements

- Arduino (Uno, Nano, Mega, etc.)
- MarvelMind SuperBeacon (connected via UART to the Arduino)
- USB connection to a host PC running ROS 2

## 🛠 Usage

1. Upload the sketch to your Arduino
2. Ensure the MarvelMind beacon is powered and connected
3. Open the serial monitor or connect the Arduino to a PC
4. Use a ROS 2 script to receive and process the data

## 💡 Tip

This sketch is optimized for integration with a ROS 2 pipeline that processes pose data via serial input — good fit for mobile robotics or indoor localization applications.

---

Developed by **Armando Nicolella** with special thanks to **Rafa Carbonell**

