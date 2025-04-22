# 🛰️ Custom Arduino Rover Control with IMU & Wi-Fi (OSOYOO App Integration)

This Arduino project enables real-time control of a six-wheel rover equipped with IMU sensors, Wi-Fi connectivity, and motor drivers (Adafruit Motor Shields). The rover can be controlled remotely via the **OSOYOO Wi-Fi Robot Car App** or through a web interface.

The system implements:
- Kalman filtering for pitch, roll, and yaw estimation
- Skid steering and differential drive logic
- Real-time logging via SD card or Serial Monitor
- Wi-Fi communication via UDP
- Modular and extensible code structure



## 📂 File Structure

| File | Description |
|------|-------------|
| `app_control.ino` | **Main entry point**. This is the core Arduino sketch that initializes sensors, handles UDP commands from the OSOYOO app, and manages movement and IMU data. |
| `Kalman.h` | Kalman filter implementation (external dependency, by TKJElectronics). Used for orientation estimation. |
| Additional `.ino` functions | All helper functions (e.g., `LevogiroPos`, `LevogiroNeg`, `SkidSteering`, `updateYaw`, `readNwriteSD`) are declared inline in the `.ino` files. |

---

## 🚘 Features

- 🧭 IMU-based orientation estimation (roll, pitch, yaw)
- 📶 Wi-Fi AP Mode using `WiFiNINA` + `UDP` commands
- ⚙️ Dual Adafruit Motor Shield support (I2C address selection)
- 🧮 Skid steering and directional control
- 💾 SD card logging (optional)
- 📱 OSOYOO App compatibility
- 🔧 Easily configurable for different

