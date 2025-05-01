# 🎯 RGB-D Vision-Based Control Scripts

This repository contains MATLAB scripts and Simulink models developed for vision-based closed-loop control using RGB-D cameras (RealSense D435) to track and stabilize the position of a red ball. The scripts are specifically designed for the project **"RGB-D Camera Controlled Linear Actuator."**

---

## 🛠️ Code Description

### 1. `Ball_Red_Center_F.m`

- **Purpose**: Acquires RGB-D data from the Intel RealSense camera and computes the position and control signals for a red ball.
- **Inputs**: None (uses global variables initialized externally)
- **Outputs**:
  - `x`: 1x2 vector `[1, distance(mm)]` from zero reference
  - `xyz_mean`: 1x3 vector representing ball center `[x, y, z]` (meters)
  - `control`: Scalar control signal
- **Dependencies**: `find_red.m`, global RealSense pipeline setup

### 2. `find_red.m`

- **Purpose**: Processes RGB-D point cloud data to detect a red ball, extracting its 3D coordinates and generating a control feedback signal.
- **Inputs**:
  - `vertices`: Nx3 point cloud coordinates
  - `colordatavector`: Nx3 RGB color data
  - `width`, `height`: Resolution of depth image
  - `rect`: Region of interest (ROI) cropping rectangle
- **Outputs**:
  - `x`: Distance feedback signal
  - `xyz_mean`: Estimated center coordinates of the red ball
  - `control`: Control signal (distance from target reference)
- **Dependencies**: `sphereFit_1.m`

### 3. `sphereFit_1.m`

- **Purpose**: Fits a sphere to a point cloud of a detected spherical object to estimate its center coordinates.
- **Inputs**:
  - `Ball_Cloud_clean`: Point cloud object of the filtered ball surface
  - `r`: Known radius of the ball (e.g., 0.020 m)
- **Outputs**:
  - Sphere center coordinates `(x1, y1, z1)`

### 4. `main_pipeline.slx`

- **Purpose**: Simulink model implementing the real-time closed-loop control algorithm for the linear actuator using vision-based feedback.
- **Execution**: Load the model into Simulink, set simulation parameters, and run.

---

## ⚙️ How to Run

### Initial Setup

1. Ensure Intel RealSense SDK for MATLAB is installed.
2. Initialize global variables and RealSense pipeline externally before running MATLAB scripts:

```matlab
pipe = realsense.pipeline();
colorizer = realsense.colorizer();
pcl_obj = realsense.pointcloud();

profile = pipe.start();
align_to = realsense.stream.color;
alignedFs = realsense.align(align_to);
```

## 📌 Important Notes
Adjust ROI (rect) and sphere radius (r) parameters according to your experimental setup.  <br>
Ensure correct calibration of RGB-D camera for accurate measurements.


## 📚 Citation
If you use this code, please cite:

```
@article{cosenza2020mechanical,
  title={Mechanical system control by RGB-D device},
  author={Cosenza, Chiara and Nicolella, Armando and Esposito, Daniele and Niola, Vincenzo and Savino, Sergio},
  journal={Machines},
  volume={9},
  number={1},
  pages={3},
  year={2020},
  publisher={MDPI}
}
```
```
@inproceedings{cosenza2020rgb,
  title={RGB-D vision device for tracking a moving target},
  author={Cosenza, Chiara and Nicolella, Armando and Niola, Vincenzo and Savino, Sergio},
  booktitle={The International Conference of IFToMM ITALY},
  pages={841--848},
  year={2020},
  organization={Springer}
}
```

👨‍💻 Developed as part of MSc Thesis @ Università degli Studi di Napoli Federico II
