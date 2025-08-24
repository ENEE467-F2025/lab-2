#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

plt.rcParams.update({"figure.figsize": (12, 9), "font.size": 20})
plt.rcParams['axes.labelsize'] = 20  
plt.rcParams['axes.titlesize'] = 20  
plt.rcParams['xtick.labelsize'] = 20 
plt.rcParams['ytick.labelsize'] = 20 
LW = 5

robot = rtb.models.DH.UR3() # Load UR3 model

q_app = np.radians([236, -167, 75, -88, -236, 0])     
q = q_app                                         # start near a ready pose
dt = 0.025                                        # sample time
N = 100                                           # number of time steps over which to execute motion
v_des_x = np.array([0.1, 0, 0])                   # 5cm/s in base X (both directions)
v_des_z = np.array([0, 0, 0.1])                   # 5cm/s in base Z (both directions)

L = 0.1  # side length of square, m
steps_per_edge = int(L / (np.linalg.norm(v_des_x) * dt))

traj = [q]
ee_pos = [robot.fkine(q).t]                      # initial end-effector position

for i in range(4 * steps_per_edge):
    J = robot.jacob0(q)[:3, :]                  # Compute Jacobian
    if i < steps_per_edge:
        dq = np.linalg.pinv(J) @ v_des_x
    elif i < 2 * steps_per_edge:
        dq = np.linalg.pinv(J) @ v_des_z
    elif i < 3 * steps_per_edge:
        dq = np.linalg.pinv(J) @ -v_des_x
    else:
        dq = np.linalg.pinv(J) @ -v_des_z

    q = q + dq * dt
    traj.append(q)
    ee_pos.append(robot.fkine(q).t)

print("final pose:\n", robot.fkine(q))
ee_arr = np.array(ee_pos)
traj_arr= np.array(traj)

# desired ee_pos for comparison
ee_start = robot.fkine(q_app).t
ee_goal_x = ee_start + np.array([L, 0, 0])
ee_goal_z = ee_goal_x + np.array([0, 0, L])
ee_goal_nx = ee_goal_z - np.array([L, 0, 0])

ee_des = np.vstack([
    np.linspace(ee_start, ee_goal_x, steps_per_edge),
    np.linspace(ee_goal_x, ee_goal_z, steps_per_edge),
    np.linspace(ee_goal_z, ee_goal_nx, steps_per_edge),
    np.linspace(ee_goal_nx, ee_start, steps_per_edge)
])

robot_fig = robot.plot(q, block=False, backend='pyplot', dt=dt, jointaxes=False) # turn off quiver for better viz
ax = robot_fig.ax
line, = ax.plot([], [], [], 'r-', linewidth=LW, label="actual", zorder=2)
ax.plot(ee_des[:, 0], ee_des[:, 1], ee_des[:, 2], 'b--', linewidth=LW, label="desired", alpha=1, zorder=1) # desired ee_traj

for i, q in enumerate(traj_arr):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    line.set_data(ee_arr[:i+1, 0], ee_arr[:i+1, 1])
    line.set_3d_properties(ee_arr[:i+1, 2])
    plt.pause(0.01)  
    ax.axis('equal')
    ax.legend()
    