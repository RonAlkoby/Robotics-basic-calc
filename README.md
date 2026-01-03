# Planar RRR Robot - Path Planning and Configuration Space Visualization

This repository contains a Python implementation for motion planning and visualization of a planar 3-link RRR robotic arm.  
The project was completed as part of an Introduction to Robotics course and addresses both global planning and reactive navigation problems.

---

## Overview

The project focuses on three main robotics challenges:

1. Configuration Space (Q_free) approximation  
2. Trajectory planning in joint space using cubic polynomials  
3. Artificial Potential Fields (APF) for reactive obstacle avoidance  

---

## Robot Specifications

### Kinematic Structure
- Planar RRR manipulator (3 revolute joints)

### Link Lengths
- l1 = 1.0 m  
- l2 = 0.8 m  
- l3 = 0.7 m  

### Joint Limits
- θ1: -180° to 180°  
- θ2: 0° to 180°  
- θ3: -180° to 180°  

### Collision Constraints
- Collision checks are applied only to the end-effector (gripper)
- Obstacles are modeled as circles in the workspace
- Vertical workspace boundaries:  
  - y ∈ [0.4, 1.2]

---


---

## Features

### 1. Configuration Space Approximation (Q_free)

- 10,000 random joint configurations are sampled
- Each configuration is validated against:
  - End-effector collision with obstacles
  - Workspace boundary constraints
- Valid configurations are stored and visualized as a 3D point cloud in joint space:
  - (θ1, θ2, θ3)

This process provides an approximate representation of the collision-free configuration space.
<img width="640" height="480" alt="Q_free_in_joint_space_2" src="https://github.com/user-attachments/assets/2b714f89-c87a-444b-8c2b-4ffbad602534" />


---

### 2. Polynomial Trajectory Planning

- A smooth joint-space trajectory is planned using cubic polynomials
- The path connects two valid configurations corresponding to:
  - Start position: (0.5, 1.0)
  - Goal position: (-1.3, 0.5)
- The resulting motion:
  - Avoids obstacles
  - Produces continuous joint angles and velocities
- Outputs include:
  - End-effector path in the workspace
  - Joint angles as a function of time

<img width="800" height="600" alt="notors angles" src="https://github.com/user-attachments/assets/61f6f44d-438f-45e9-9f8b-99db4505323c" />

<img width="640" height="480" alt="track_task2" src="https://github.com/user-attachments/assets/49163e4c-a3b0-4bd1-ab84-eb7ebc4ed94d" />

---

### 3. Artificial Potential Fields (APF)

A reactive navigation strategy is implemented using artificial potential fields:

- Attractive potential pulls the robot toward the goal
- Repulsive potential pushes the robot away from obstacles and workspace boundaries
- The robot follows the negative gradient of the total potential field
- The simulation continues until convergence at the goal or a stopping condition is reached

This implementation demonstrates typical APF behavior, including sensitivity to local minima.
<img width="640" height="480" alt="track_task3" src="https://github.com/user-attachments/assets/5177cc98-b83f-4ab6-9f41-620798961f4d" />

---

## Installation and Requirements

### Requirements
- Python 3.x
- numpy
- matplotlib

### Installation

pip install numpy matplotlib


---

## Usage

### Configuration Space and Polynomial Trajectory Planning

python robotics_calc_1_and_2.py


### Artificial Potential Field Simulation
python robotics_calc_3.py


---


