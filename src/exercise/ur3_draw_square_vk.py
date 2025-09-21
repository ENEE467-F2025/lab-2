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

def compute_vk_err(ref, curr_q, curr_dq, v_or_w: str):
    J = robot.jacob0(curr_q)
    if v_or_w == "v":
        J_part = J[:3, :]
    elif v_or_w == "w":
        J_part = J[3:, :]
    actual = J_part @ curr_dq
    err = np.linalg.norm(actual - ref)
    return err

q_app = np.radians([236, -167, 75, -88, -236, 0])     
q = q_app                                         # start near a ready pose
dt = 0.025                                        # sample time
N = 100                                           # number of time steps over which to execute motion
v_des_x = np.array([0.1, 0, 0])                   # 10cm/s in base X 
v_des_z = np.array([0, 0, 0.1])                   # 10cm/s in base Z 

L = 0.1  # side length of square, m
steps_per_edge = int(L / (np.linalg.norm(v_des_x) * dt))

traj = [q]
ee_pos = [robot.fkine(q).t]                      # initial end-effector position
v_err = [compute_vk_err(v_des_x, q, np.zeros_like(q), 'v')] 
omega_err = [compute_vk_err(np.array([0, 0, 0]), q, np.zeros_like(q), 'w')]

###### TODO: Ex. 2 (Adapt velocity update logic to maintain zero angular velocity) ############
for i in range(4 * steps_per_edge):
    J = robot.jacob0(q)[:3, :]                  # Compute Jacobian
    if i < steps_per_edge:
        vref = v_des_x
    elif i < 2 * steps_per_edge:
        vref = v_des_z
    elif i < 3 * steps_per_edge:
        vref = -v_des_x
    else:
        vref = -v_des_z
    dq = np.linalg.pinv(J) @ vref

    q = q + dq * dt
    traj.append(q)
    ee_pos.append(robot.fkine(q).t)

    v_err.append(compute_vk_err(vref, q, dq, 'v'))
    omega_err.append(compute_vk_err(np.array([0, 0, 0]), q, dq, 'w'))
###### TODO: Ex. 2 (Adapt velocity update logic to maintain zero angular velocity) ############

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
mse_text = ax.text2D(0.2, 0.25, mse_annot_v + "" + mse_annot_w, 
                     transform=ax.transAxes, fontsize=22, verticalalignment='top')

for i, q in enumerate(traj_arr):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    line.set_data(ee_arr[:i+1, 0], ee_arr[:i+1, 1])
    line.set_3d_properties(ee_arr[:i+1, 2])
    plt.pause(0.01)  
    ax.axis('equal')
    ax.legend()
    