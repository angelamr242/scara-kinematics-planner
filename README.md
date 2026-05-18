# 3-DOF SCARA Manipulator Trajectory & Kinematics Planner

An analytical mechatronics project implementing an absolute geometric Forward/Inverse Kinematics solver with an optimized multi-axis Trapezoidal Velocity Profile Planner.

## Project Architecture & Core Features
* **Kinematics Engine:** Custom implementation of analytical transformations mapping Cartesian Space coordinates directly to Joint Space positions ($\theta_1, \theta_2, d_3$).
* **Motion Profile Optimization:** Joint space path generation executing smooth acceleration and deceleration curves to eliminate mechanical jerk.
* **Virtual Testing Framework:** Built-in real-time 3D simulation engine mapping closed-loop path execution tasks.



## Mathematical Formula Definitions
The forward analytical workspace coordinates are derived explicitly via structural Denavit-Hartenberg (DH) parameters:

* $X = a_1 \cdot \cos(\theta_1) + a_2 \cdot \cos(\theta_1 + \theta_2)$
* $Y = a_1 \cdot \sin(\theta_1) + a_2 \cdot \sin(\theta_1 + \theta_2)$
* $Z = -d_3$

## Execution Guide
Execute the entire simulation pipeline natively using these terminal commands:

```bash
git clone [https://github.com/angelamr2424/scara-kinematics-planner.git](https://github.com/angelamr242/scara-kinematics-planner.git)
cd scara-kinematics-planner
pip install numpy matplotlib
python3 simulate.py
