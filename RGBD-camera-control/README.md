# 🧠 RGB-D Camera Controlled Linear Actuator

This project demonstrates how to use an RGB-D camera to measure and control the position of a linear actuator with no physical contact or feedback sensors. The system leverages real-time depth data to track motion and issue control commands, enabling a flexible and non-invasive actuation strategy.

---

## 🎯 Objective

- Accurately **measure the actuator's displacement** through computer vision techniques.
- Implement a **feedback control loop** where actuator movement is adjusted based on visual input.
- Test applications such as **target reaching** and **ball balancing** using visual feedback only.

---

## ⚙️ Hardware Components

| Component | Specs |
|----------|-------|
| **Intel® RealSense D435** | RGB-D camera, up to 1280×720 @ 90 fps |
| **Microsoft Kinect V2** | RGB-D camera, 512×424 depth @ 30 fps |
| **Pololu® LACT4P-12V-05** | 10 cm stroke, 150 N, 12 V linear actuator |
| **Pololu® JRK 21v3** | USB/DC motor controller (supports feedback and no-feedback modes) |

---

## 🧪 Experimental Setup

The test bench includes:
- RGB-D cameras placed to observe the actuator from different angles.
- The actuator mounted with visible markers or directly processed through 3D point clouds.
- Lighting variation tests to ensure robustness.

📷 *Example setup image*: `./docs/setup_d435.jpg`

---

## 🔍 Measurement Techniques

### 🎨 Marker-Based Measurement
- **Color segmentation** (Red/Blue)
- Compute distances between marker centroids or relative to initial position

### 🧱 Markerless Measurement (Preferred)
- **3D point cloud extraction**
- **Cylinder fitting algorithm**
  - Fit a cylinder to the actuator shaft
  - Collapse normals toward the axis
  - Estimate displacement from endcap distance

📌 *Best results achieved with D435 + markerless method*  
✅ High repeatability  
✅ Insensitive to lighting changes  
✅ No physical contact required

---

## 🎮 Control Applications

### 🔴 Hit-the-Target
- Vision-based closed-loop to bring the actuator to a predefined visual marker.

### ⚖️ Ball Balance System
- Control a slide's inclination via actuator to balance a ball at a desired position.

Implemented strategies:
- **Midpoint control**: Distance from ball to visual center
- **Zero-point PI control**:
  - `Kp = 150`, `Ki = 2`
  - Gain tuning: tested values `0.1`, `0.09`, `0.08`

Control loop:
[ Acquisition Block ] → [ Control Block ] → [ Command Block ]


📷 *Example block diagram*: `./docs/control_block_diagram.png`

---

## 📊 Test Results Summary

| Test Type | Precision | Notes |
|-----------|-----------|-------|
| Red & Blue Markers | Medium | Error from dual marker offset |
| Red Marker Only | Good | Simpler, less error accumulation |
| Markerless (3D Fit) | Excellent | Best accuracy and repeatability |

---

## ✅ Conclusions

- Vision systems can **precisely measure actuator displacement** with no sensors.
- Markerless 3D-based methods show **higher robustness and accuracy**.
- Visual feedback enables control tasks like object targeting and balance maintenance.

---

## 🚀 Future Work

- Speed up visual processing and control latency
- Test smarter control architectures (MPC, RL, etc.)
- Port the system to different actuators or robotic platforms

---

## 🧾 References

- Nicolella, A. *"Vision Techniques to Control Mechanical Systems"* – MSc Thesis, Università degli Studi di Napoli Federico II
- [Pololu JRK Controllers](https://www.pololu.com/product/1393)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [Kinect SDK](https://learn.microsoft.com/en-us/windows/kinect/)

---

## 📂 Repository Structure


