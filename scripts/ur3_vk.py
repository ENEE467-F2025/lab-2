#!/usr/bin/env python3
import os
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import matplotlib.pyplot as plt
import matplotlib.animation as animation

robot = rtb.models.DH.UR3() # Load UR3 model

q_app = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]     
q = q_app                                      # start near a ready pose
dt = 0.02                                       # sample time
N = 200                                         # number of time steps over which to execute motion
v_des = np.array([0.05, 0, 0])                   # 5cm/s in base X
traj = [q]
ee_pos = [robot.fkine(q).t]                      # initial end-effector position
for _ in range(200):
    J = robot.jacob0(q)[:3, :]                   # only need linear part of J (i.e., J_v) to update v
    dq = np.linalg.pinv(J) @ v_des               # corresponding joint velocity
    q = q + dq * dt                              # update joint velocity
    traj.append(q)                               # keep track of q for animation
    ee_pos.append(robot.fkine(q).t)              # keep track of ee pos for anim

print("final pose:\n", robot.fkine(q))
ee_arr = np.array(ee_pos)
traj_arr= np.array(traj)

# desired ee_pos for comparison
ee_start = robot.fkine(q_app).t
ee_goal = ee_start + v_des * dt * N
ee_des = np.linspace(ee_start, ee_goal, N)

robot_fig = robot.plot(q, block=False, backend='pyplot', dt=dt, jointaxes=False) # turn off quiver for better viz
ax = robot_fig.ax
line, = ax.plot([], [], [], 'r-', linewidth=2, label="actual")
ax.plot(ee_des[:, 0], ee_des[:, 1], ee_des[:, 2], 'b--', linewidth=2, label="desired") # desired ee_traaj

for i, q in enumerate(traj_arr):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    line.set_data(ee_arr[:i+1, 0], ee_arr[:i+1, 1])
    line.set_3d_properties(ee_arr[:i+1, 2])
    plt.pause(0.01)  
    ax.legend()
    