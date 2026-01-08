# trajectory_control_foundation

The project is based on **Homework 1 of AA174A – Principles of Robot Autonomy

## Project Overview

This project demonstrates:

1. Robot dynamics modeling and simulation  
2. Trajectory generation using differential flatness  
3. Feedback control for trajectory tracking  
4. Deployment and validation in a ROS2 simulation environment  

##  Repository Structure

trajectory_control_foundation/
├── README.md
├── .gitignore
│
├── python_hw/ # Problem 1 & 2: dynamics, planning, control
│ ├── P1_dynamics/
│ └── P2_differential_flatness/
│ └── plots/
│ ├── differential_flatness.png
│ ├── sim_traj_openloop.png
│ └── sim_traj_closedloop.png
│
└── autonomy_ws/ # Problem 3: ROS2 system integration
└── src/
├── autonomy_repo/
│ └── plots/
│ └── heading_control_response.png
└── asl_tb3_sim/


## 🧠 Part I – Robot Dynamics and Simulation (Python)

**Key topics**
- Discrete-time kinematic modeling of a differential-drive robot
- Euler integration
- Multi-trajectory rollouts
- Vectorized simulation for performance

**Implemented models**
- TurtleBot (unicycle model)
- Double integrator dynamics (vectorized rollouts)

This part focuses on translating mathematical models into efficient and reusable simulation code.

---

## 🧭 Part II – Trajectory Generation via Differential Flatness

**Key topics**
- Differential flatness for unicycle robots
- Polynomial trajectory generation with boundary constraints
- Mapping flat outputs to control inputs \(v, \omega\)
- Trajectory tracking using PD feedback

---

### Nominal Trajectory and Control Inputs

The following figure shows the nominal trajectory generated using differential flatness
and the corresponding control inputs.

<img width="1438" height="706" alt="image" src="https://github.com/user-attachments/assets/dbb5b567-4abf-4770-91bb-e8a98ab3cc69" />

---

### Open-Loop Execution

When executed in open loop, the robot deviates noticeably from the nominal trajectory due to
model mismatch and accumulated errors.

<img width="1278" height="636" alt="image" src="https://github.com/user-attachments/assets/ed333da1-6ca7-4e79-832b-36d8e317a496" />


---

### Closed-Loop Trajectory Tracking

With feedback control enabled, the robot tracks the desired trajectory closely,
significantly reducing deviation and improving robustness.

<img width="1279" height="637" alt="image" src="https://github.com/user-attachments/assets/ab3f69f3-f75d-46b2-9eb6-5d8052cc388d" />


---

## 🤖 Part III – ROS2 Heading Controller and System Integration

**Key topics**
- ROS2 node implementation
- Heading control using proportional feedback
- Launch files and parameter handling
- Headless simulation debugging (GPU-independent)

The heading controller is deployed in a ROS2 simulation environment and commands
the robot to align its heading with a desired target orientation.

---

### Heading Control Response

The following plot shows smooth and stable convergence of the robot’s heading
to the desired goal angle.

<img width="796" height="598" alt="image" src="https://github.com/user-attachments/assets/3118c264-5520-4b4e-b72a-3c6d4c89a0d4" />


---

## How to Run

### Requirements
- Ubuntu 22.04
- ROS2 Humble
- Python ≥ 3.10
- NumPy, SciPy, Matplotlib

---

### Build the ROS2 Workspace
- cd trajectory_control_foundation/autonomy_ws
- colcon build
- source install/local_setup.bash

### Launch the simulator (terminal 1)
- cd trajectory_control_foundation/autonomy_ws
- source install/local_setup.bash
- export LIBGL_ALWAYS_SOFTWARE=1
- ros2 launch asl_tb3_sim root.launch.py gui:=false

### Running the heading controller (terminal 2)
- cd trajectory_control_foundation/autonomy_ws
- source install/local_setup.bash
- ros2 launch autonomy_repo heading_control.launch.py

### Generate evaluation plot (terminal 3)
- cd trajectory_control_foundation/autonomy_ws
- source install/local_setup.bash
- ros2 run autonomy_repo p3_plot.py

