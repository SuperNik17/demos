# H-infinity Control for Robust Trajectory Tracking

##  Motivation

Trajectory tracking is a fundamental requirement for autonomous mobile robots operating in uncertain and dynamic environments.  
Classical control methods, while effective under ideal conditions, often fail to deliver reliable performance when faced with disturbances, sensor noise, or unmodeled dynamics.

To overcome these challenges, this repository implements an **H∞ control strategy** that enhances robustness and ensures high-precision tracking even under adverse conditions.

##  Control Strategy Overview

The control architecture is based on two key components:

- **Feedforward Control**: Generates the desired trajectory assuming no disturbances.  
- **Feedback Control**: Reacts to external disturbances and model inaccuracies to minimize trajectory errors.

The H∞ controller is designed to **minimize the worst-case amplification** of disturbances.  
This is achieved through an optimization process based on **Linear Matrix Inequalities (LMIs)**, ensuring both stability and performance.

<p align="center">
  <img src="img/controller_diagram2.png" alt="Controller Block Diagram" width="600"/>
</p>
<p align="center"><em>Block diagram of the control system, including inverse kinematics, feedforward and feedback components.</em></p>

##  Experimental Validation

The proposed controller has been validated on a **TurtleBot3 Waffle Pi** using **ROS 2**.  
Experiments involved tracking a figure-eight (∞) trajectory in an indoor environment with sensor noise and external disturbances.
<p align="center">
  <img src="img/h_inf_setup_short.gif" alt="Setup" width="700"/>
</p>
<p align="center"><em>The real-world testbed.</em></p>

The localization system is based on the Marvelmind Indoor "GPS" beacons, connected via an Arduino board to ROS 2.  
The setup includes serial communication publishing position data as ROS topics.
<p align="center">
  <img src="img/localization_setup.png" alt="Localization Setup" width="700"/>
</p>
<p align="center"><em>Localization system architecture using Marvelmind and ROS 2.</em></p>


##  Performance Metrics

| Controller           | X RMS Error [m] | Y RMS Error [m] |
|----------------------|------------------|------------------|
| P (filter)           | 0.0032           | 0.0027           |
| PID                  | 0.0031           | 0.0024           |
| Full Order (H∞)      | **0.0028**       | **0.0021**       |


<p align="center">
  <img src="img/exp_trajpng.png" alt="Experimental Trajectory Tracking" width="600"/>
</p>
<p align="center"><em>Experimental trajectory tracking performance for different controllers.</em></p>

<p align="center">
  <img src="img/wheel_speeds.png" alt="Wheel Speeds Comparison" width="600"/>
</p>
<p align="center"><em>Comparison of wheel speeds for feedforward and feedback control components.</em></p>


##  Copyright & Acknowledgments

This work was carried out during my PhD at the **Universitat Politècnica de València**.
Special thanks to: **Prof. Leopoldo Armesto**, for his continuous guidance and scientific supervision, **Ricardo Núñez Sáez**, for his contribuition and insightful discussions throughout the project.  

**Main scientific reference** for this repository:

> R. Núñez, A. Nicolella, L. Armesto, A. González, A. Sala.  
> *Diseño de controladores H-infinito para el seguimiento de trayectorias con robots móviles con ruedas*.  
> Jornadas de Automática, No. 45, 2024.

### BibTeX
```bibtex
@article{nunez2024diseno,
  title={Dise{\~n}o de controladores H-infinito para el seguimiento de trayectorias con robots m{\'o}viles con ruedas},
  author={N{\'u}{\~n}ez, Ricardo and Nicolella, Armando and Armesto, Leopoldo and Gonz{\'a}lez, Antonio and Sala, Antonio},
  journal={Jornadas de Autom{\'a}tica},
  number={45},
  year={2024}
}

