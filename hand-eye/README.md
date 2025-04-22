# 🎯 Hand–Eye Calibration for Mobile Robot Manipulation

This module presents a MATLAB-based implementation of a **marker-based hand–eye calibration** pipeline between an external camera and its end-effector, fundamental for **Position-Based Visual Servoing (PBVS)**. The system is designed to work with **unstructured fiducial markers** (such as custom patterns), offering flexibility across different platforms and camera models.

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

## 🖼️ System Architecture

<p align="center">
  <img src="PBVS_servoing.png" alt="Frame composition diagram - Hand–Eye Calibration" width="500">
</p>

<p align="center">
  <sub><sup>
    Frame graph showing the reference transformations between:  
    base frame {0}, end-effector {E}, gripper {G}, marker, and camera {C}.  
    Calibration aims to resolve the unknown transformation between \({C}\) and \({0}\) through pose estimation.
    <br><br>
    <em>Diagram adapted from:</em>  
    Peter Corke, <em>Robotics, Vision and Control: Fundamental Algorithms in MATLAB®</em>, Springer, 2011.  
    © Springer-Verlag London Limited 2011. All rights reserved.
  </sup></sub>
</p>



## 📐 Transformation Composition
<p align="center"> <img src="/tree_T.png" alt="Transformation graph - reference frame composition" width="420"> </p> <p align="center"> 
  <sub><sup> Graph showing the composition of transformations between reference frame (*r*), marker frame (*M*), and camera frame (*c*). The goal of the calibration is to estimate the unknown transformation <strong>\(    
    {}^r\mathbf{T}_c \)</strong> from the camera frame to the reference. 
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
