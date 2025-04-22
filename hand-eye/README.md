# 🎯 Hand–Eye Calibration for Mobile Robot Manipulation

This module presents a MATLAB-based implementation of a **marker-based hand–eye calibration** pipeline between an external camera and its end-effector. The system is designed to work with **unstructured fiducial markers** (such as custom patterns), offering flexibility across different platforms and camera models.

---

## 🧠 Project Overview
The pipeline is designed for a **mobile robotic platform** equipped with a fixed onboard RGB-D camera. However, all current **experiments were performed off-board**, without an actual mobile base. The camera is positioned to capture both the **end-effector** and a portion of the **surrounding environment**, ensuring spatial observability for hand–eye calibration.

- **Objective:** Estimate the transformation matrix `X` in the equation `AX = XB`
- **Setup:**  
  - Mobile base with fixed onboard camera for perception.
  - Robotic arm or mock end-effector  
  - Fiducial markers for reference frame estimation
- **Implementation:** Developed in **MATLAB**, using rigid body transformations and vision-based pose estimation  
- **Marker:** Non-structured — supports custom patterns if implemented with segmentation  
- **Use case:** Moving object in the robot workspace
## 🖼️ System Architecture

<p align="center">
  <img src="fPBVS_servoing.png.png" alt="Frame composition diagram - Hand–Eye Calibration" width="500">
</p>

<p align="center">
  <sub><sup>
    Frame graph showing the reference transformations between:  
    base frame \({0}\), end-effector \({E}\), gripper \({G}\), marker, and camera \({C}\).  
    Calibration aims to resolve the unknown transformation between \({C}\) and \({0}\) through pose estimation.
  </sup></sub>
</p>


## 🧰 Tools & Components

| Component            | Tool / Device                          |
|----------------------|----------------------------------------|
| RGB-D Camera         | Intel Realsense D415                   |
| Manipulator          | Dobot Magician                         |
| Marker type          | Custom planar marker (non-structured)  |
| Camera interface     | MATLAB                        |
| Marker detection     | Custom detection (no ArUco required)   |
| Calibration Optimization solver   | Least Squares  |
| Toolbox              | MATLAB Computer Vision Toolbox         |         
| Data logging         | CSV logs               |




## 🛠️ Development Status

> ⚠️ This module is currently under **active development**.  
> Code is being validated and optimized; full open-source release is not yet available.  
> For academic collaboration or review purposes, access may be granted upon request.
