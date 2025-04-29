# 🧠 RGB-D Camera Controlled Linear Actuator

This project demonstrates how to use an RGB-D camera to measure and control the position of a linear actuator in a completely contactless fashion. The actuator is controlled in closed-loop using only computer vision feedback. The approach leverages both color segmentation and point-cloud geometry to track motion and enable basic mechanical system control.

---

## 🌟 Key Goals

- Measure actuator displacement using **depth cameras** (no encoders or potentiometers)
- Implement **vision-based control** for:
  - Reaching a visual target ("Hit Red")
  - Stabilizing a ball on a moving slide ("Ball Control")

---

## ⚙️ Hardware Setup

| Component                    | Description                                      |
|-----------------------------|--------------------------------------------------|
| **Intel® RealSense D435**   | RGB-D camera (active stereo, 1280x720 @90 fps)   |
| **Microsoft Kinect V2**     | RGB-D camera (structured light, 512x424 @30 fps) |
| **Pololu LACT4P-12V-05**    | Linear actuator (10 cm stroke, 150 N)           |
| **Pololu JRK 21v3**         | USB motor controller (with/without feedback)     |

---

## 🔧 Software Stack

- **Vision processing**: depth stream acquisition, segmentation, and point-cloud geometry
- **Control loop**: implemented in MATLAB/Simulink for closed-loop actuation

---

## 📊 Measurement Techniques

### 1. Marker-Based
- Color segmentation to detect red or blue markers
- Compute Euclidean distance from initial reference or between two markers

### 2. Markerless (Best Performance)
A cylinder fitting algorithm is applied to the 3D point cloud representing the actuator shaft:

#### 📄 Algorithm Steps:
1. **Acquire** 3D point cloud of the actuator
2. **Compute** surface normals and flip them toward the cylinder axis
3. **Project** points along normals toward axis
4. **Fit a line** to projected points (least squares)
5. **Measure** actuator stroke as distance between two farthest points on the axis

![Plane Fit Algorithm](./docs/plane_fit_algorithm.png)

> Reference: /cite{Mechanical_System_Control_by_RGB_D_Device}

**Performance:**
- Average distance error: **2.83 mm**
- Standard deviation: **0.35 mm** across 5 tests

---

## 🎓 Control Applications

### 🎈 1. Target Tracking ("Hit Red")
- Use color segmentation to detect a red cylinder
- Compute 3D position from RGB-D data
- Drive actuator to target using vision feedback
- Implemented in Simulink using closed-loop control

### ⚖️ 2. Ball Stabilization ("Ball Control")
- Detect red ball using color + sphere fitting
- Control actuator to balance the ball at a fixed point
- Predict motion via discrete derivative (velocity estimate)
- Feed-forward + PI Control (`Kp = 150`, `Ki = 2`)
- Tested gain values: `0.1`, `0.09`, `0.08`

#### Key Result:
- Stable equilibrium achieved within **8-11 mm** from target despite mechanical friction and sensor noise.

---

## 📆 Experimental Repeatability

- **10 actuator positions** tested from 0 to 100 mm
- **5 repetitions** per position
- Markerless method proved most robust

---

## 📄 Repository Structure

```
RGBD-camera-control/
├── Linear Actuator control by RGBD Camera/
│   ├── src/                   # Code for vision and control (MATLAB/Python)
│   ├── docs/                  # Schematics, algorithm illustrations, images
│   └── README.md              # Project description
```

---

## 📅 Future Work
- Real-time control optimization
- Integrate adaptive control strategies (e.g., MPC, reinforcement learning)
- Apply to more complex robotic systems

---

## 📅 Citation
If you use this project, please cite:

```
@article{nicolella_rgbd_2021,
  title={Mechanical System Control by RGB-D Device},
  author={Cosenza, Chiara and Nicolella, Armando and Esposito, Daniele and Niola, Vincenzo and Savino, Sergio},
  journal={Machines},
  volume={9},
  number={1},
  pages={3},
  year={2021},
  publisher={MDPI},
  doi={10.3390/machines9010003}
}
```

---

📈 Developed as part of Armando Nicolella's MSc thesis at Università degli Studi di Napoli Federico II.

---


