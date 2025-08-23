#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams.update({"figure.figsize": (12, 9), "font.size": 20})
plt.rcParams['axes.labelsize'] = 20  
plt.rcParams['axes.titlesize'] = 20  
plt.rcParams['xtick.labelsize'] = 20 
plt.rcParams['ytick.labelsize'] = 20 

LW = 5
robot = rtb.models.DH.UR3() # Load UR3 model

def compute_vk_err(ref, curr_q, curr_dq, v_or_w: str):
    J = robot.jacob0(curr_q)
    if v_or_w == "v":
        J_part = J[:3, :]
    elif v_or_w == "w":
        J_part = J[3:, :]
    actual = J_part @ curr_dq
    err = np.linalg.norm(actual - ref)
    return err

q_app = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]     
q = q_app                                        # start near a ready pose
dt = 0.02                                        # sample time
N = 200                                          # number of time steps over which to execute motion
v_des = np.array([0.05, 0, 0])                   # 5cm/s in base X
traj = [q]
ee_pos = [robot.fkine(q).t]                      # initial end-effector position
v_err = [compute_vk_err(v_des, q, np.zeros_like(q), 'v')] 
omega_err = [compute_vk_err(np.array([0, 0, 0]), q, np.zeros_like(q), 'w')]

for _ in range(200):
    ###### MODIFY ############
    J = robot.jacob0(q)[:3, :]                   # Compute J
    dq = np.linalg.pinv(J) @ v_des               # Compute config velocity update
    ###### MODIFY ############
    q = q + dq * dt                              # update joint velocity
    traj.append(q)                               # keep track of q for animation
    ee_pos.append(robot.fkine(q).t)              # keep track of ee pos for animation
    v_err.append(compute_vk_err(v_des, q, dq, 'v'))
    omega_err.append(compute_vk_err(np.array([0, 0, 0]), q, dq, 'w'))

print("final pose:\n", robot.fkine(q))
ee_arr = np.array(ee_pos)
traj_arr= np.array(traj)
v_err_arr = np.array(v_err)
omega_err_arr = np.array(omega_err)
v_err_mse = np.mean(np.sum(np.square(v_err_arr)))
omega_err_mse = np.mean(np.sum(np.square(omega_err_arr)))
mse_annot_v = f"MSE (v): {v_err_mse:.2e} \n"
mse_annot_w = f"MSE (omega): {omega_err_mse:.2f}"

# desired ee_pos for comparison
ee_start = robot.fkine(q_app).t
ee_goal = ee_start + v_des * dt * N
ee_des = np.linspace(ee_start, ee_goal, N)

robot_fig = robot.plot(q, block=False, backend='pyplot', dt=dt, jointaxes=False) # turn off quiver for better viz
ax = robot_fig.ax
line, = ax.plot([], [], [], 'r-', linewidth=LW, label="actual")
ax.plot(ee_des[:, 0], ee_des[:, 1], ee_des[:, 2], 'b--', linewidth=LW, label="desired") # desired ee_traj
mse_text = ax.text2D(0.2, 0.25, mse_annot_v + "" + mse_annot_w, 
                     transform=ax.transAxes, fontsize=22, verticalalignment='top')

for i, q in enumerate(traj_arr):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    line.set_data(ee_arr[:i+1, 0], ee_arr[:i+1, 1])
    line.set_3d_properties(ee_arr[:i+1, 2])
    plt.pause(0.01)  
    ax.legend()
    