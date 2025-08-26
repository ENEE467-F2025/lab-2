import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

# plots
plt.rcParams.update({"figure.figsize": (12, 9), "font.size": 20})
plt.rcParams['axes.labelsize'] = 20  
plt.rcParams['axes.titlesize'] = 20  
plt.rcParams['xtick.labelsize'] = 20 
plt.rcParams['ytick.labelsize'] = 20 
LW = 5

robot = rtb.models.DH.UR3()   
q0 = np.radians([236, -167, 75, -88, -236, 0])   
q0_copy = q0.copy() 
ee_pose_des_t = robot.fkine(q0_copy).t
ee_pose_des_R = robot.fkine(q0_copy).R
L = 0.1  # side length of square, m
dt = 0.02   # sample time
steps_per_edge = int(L / (np.linalg.norm([0.05, 0, 0]) * dt))

ee_start = robot.fkine(q0).t
ee_goal_x = ee_start + np.array([L, 0, 0])
ee_goal_z = ee_goal_x + np.array([0, 0, L])
ee_goal_nx = ee_goal_z - np.array([L, 0, 0])
square = np.vstack([
    np.linspace(ee_start, ee_goal_x, steps_per_edge),
    np.linspace(ee_goal_x, ee_goal_z, steps_per_edge),
    np.linspace(ee_goal_z, ee_goal_nx, steps_per_edge),
    np.linspace(ee_goal_nx, ee_start, steps_per_edge)
])

# Define square corners in SE(3)
###### MODIFY ############
waypoints = []
###### MODIFY ############

traj = [q0_copy]

for T in waypoints:
    ###### MODIFY ############
    sol = None
    ###### MODIFY ############
    if sol.success:
        q_sol = sol.q
        traj.append(q_sol)
        q0 = q_sol  # warm start next solve
    else:
        print("IK failed for waypoint:", T)

# Animate trajectory
traj_arr = np.array(traj)
print("--------------------------------------")
print("IK Solution:")
print("--------------------------------------\n")
print(f"{np.round(traj_arr, 2)}")

robot_fig = robot.plot(q0_copy, block=False, backend='pyplot', dt=dt, jointaxes=False) # turn off quiver for better viz
ax = robot_fig.ax
line, = ax.plot([], [], [], 'r-', linewidth=LW, label="actual", zorder=2)
ax.plot(square[:, 0], square[:, 1], square[:, 2], 'b--', linewidth=LW, label="desired", zorder = 1) # desired ee_traj
ee_actual = []  # list to store actual EE poses

for i, q in enumerate(traj_arr):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    # compute FK for current pose
    T_ee = robot.fkine(q)
    ee_actual.append(T_ee.t)

    # update line with all past ee poses
    ee_path = np.array(ee_actual)
    line.set_data(ee_path[:, 0], ee_path[:, 1])
    line.set_3d_properties(ee_path[:, 2])

    plt.pause(0.5)  
    ax.legend()
