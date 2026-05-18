import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from kinematics import ScaraKinematics
from trajectory_planner import generate_trapezoidal_profile

# 1. Initialize Robot
robot = ScaraKinematics(a1=0.5, a2=0.4)

# 2. Define Task Waypoints [X, Y, Z]
pick_point = [0.4, 0.3, -0.1]
place_point = [-0.3, 0.5, -0.2]

# 3. Calculate Target Joint Angles using Inverse Kinematics
start_joints = robot.inverse_kinematics(*pick_point)
goal_joints = robot.inverse_kinematics(*place_point)

# 4. Generate Smooth Trajectories for each joint
_, t1_path = generate_trapezoidal_profile(start_joints[0], goal_joints[0], max_vel=1.0, max_accel=1.5)
_, t2_path = generate_trapezoidal_profile(start_joints[1], goal_joints[1], max_vel=1.0, max_accel=1.5)
_, d3_path = generate_trapezoidal_profile(start_joints[2], goal_joints[2], max_vel=0.5, max_accel=1.0)

# Ensure all arrays are the exact same length for the animation loop
min_len = min(len(t1_path), len(t2_path), len(d3_path))
t1_path, t2_path, d3_path = t1_path[:min_len], t2_path[:min_len], d3_path[:min_len]

# 5. Configure the 3D Plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-0.5, 0.5])
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title("3-DOF SCARA Trajectory Execution")

# This line object represents our robot arm
line, = ax.plot([], [], [], 'ro-', lw=5, markersize=8)

# 6. The Animation Loop
def update(frame):
    # Grab the joint angles for this specific split-second in time
    th1 = t1_path[frame]
    th2 = t2_path[frame]
    d3 = d3_path[frame]
    
    # Run Forward Kinematics to find where the elbows and tool are
    x0, y0, z0 = 0, 0, 0  # Base origin
    
    # End of Link 1 (Elbow)
    x1 = robot.a1 * np.cos(th1)
    y1 = robot.a1 * np.sin(th1)
    z1 = 0
    
    # End of Link 2 (Wrist)
    x2 = x1 + robot.a2 * np.cos(th1 + th2)
    y2 = y1 + robot.a2 * np.sin(th1 + th2)
    z2 = 0
    
    # End of Tool (Prismatic joint plunging down)
    x3, y3, z3 = x2, y2, -d3 
    
    # Update the drawn line with the new coordinates
    line.set_data_3d([x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3])
    return line,

# Run the animation at 20ms per frame
ani = animation.FuncAnimation(fig, update, frames=min_len, interval=20, blit=True)
plt.show()
