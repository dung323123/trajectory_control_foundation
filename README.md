# trajectory_control_foundation

The project is based on **Homework 1 of AA174A – Principles of Robot Autonomy

## Project Overview

This project demonstrates:

1. Robot dynamics modeling and simulation  
2. Trajectory generation using differential flatness  
3. Feedback control for trajectory tracking  
4. Deployment and validation in a ROS2 simulation environment  

##  Repository Structure

<img width="626" height="455" alt="image" src="https://github.com/user-attachments/assets/e63f56c6-40f1-487d-83d1-78e1b4dab718" />


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

This figure shows the nominal trajectory generated using differential flatness.

- Left: Planned path in the 𝑥-y plane connecting the start and goal states with smooth curvature.

- Right: Corresponding control inputs 𝑉(𝑡) (linear velocity) and ω(t) (angular velocity).
The result demonstrates smooth, dynamically feasible trajectories that satisfy boundary conditions on position, velocity, and heading.
---

### Open-Loop Execution

When executed in open loop, the robot deviates noticeably from the nominal trajectory due to
model mismatch and accumulated errors.

<img width="1278" height="636" alt="image" src="https://github.com/user-attachments/assets/ed333da1-6ca7-4e79-832b-36d8e317a496" />

This plot illustrates the robot executing the planned trajectory without feedback control.

- Significant deviation from the nominal path can be observed due to model mismatch and disturbances.

- Although the control inputs follow the planned commands, the final pose error is large.
This highlights the limitation of open-loop control for real robotic systems.
---

### Closed-Loop Trajectory Tracking

With feedback control enabled, the robot tracks the desired trajectory closely,
significantly reducing deviation and improving robustness.

<img width="1279" height="637" alt="image" src="https://github.com/user-attachments/assets/ab3f69f3-f75d-46b2-9eb6-5d8052cc388d" />

This figure shows trajectory execution using a feedback tracking controller based on differential flatness.

- The closed-loop trajectory closely follows the nominal path.

- Position and heading errors are significantly reduced compared to open-loop execution.

- Control inputs remain bounded and smooth.
This confirms the effectiveness of feedback control for robust trajectory tracking.

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

This plot compares the robot’s heading θ(t) with the desired heading over time.

- The heading converges smoothly to the target value.

- No oscillation or instability is observed.
This demonstrates stable and well-tuned heading control.
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

