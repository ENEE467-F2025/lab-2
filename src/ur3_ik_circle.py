import roboticstoolbox as rtb 
import numpy as np # library containing important kernels for matrix math
import spatialmath as sm # library containing important kernels for tf math
import matplotlib.pyplot as plt

# Setup
NUM_SAMP_PTS = 20
NUM_PTS_TSP_CIRC = 200
RADIUS = 0.1 # m
dt = 0.1

# Plotting
plt.rcParams.update({"figure.figsize": (12, 9), "font.size": 20})
plt.rcParams['axes.labelsize'] = 20  
plt.rcParams['axes.titlesize'] = 20  
plt.rcParams['xtick.labelsize'] = 20 
plt.rcParams['ytick.labelsize'] = 20 

LW = 5

# Load a UR3 model
robot = rtb.models.DH.UR3() # UR3 class instance

q_des = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0] #[-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]   
ee_pose_des_t = robot.fkine(q_des).t
ee_pose_des_R = robot.fkine(q_des).R

# Create a circle in task space (ZX plane; y constant)
r = RADIUS
t = np.linspace(0, 2*np.pi, NUM_PTS_TSP_CIRC) # controls smoothness
x = r * np.cos(t)
y = ee_pose_des_t[1] * np.ones_like(x)
z = r * np.sin(t)
tsp_circle = np.vstack((x, y, z)).T

# Sample points on circle
indices = np.linspace(0, NUM_PTS_TSP_CIRC - 1, NUM_SAMP_PTS, dtype=int)
sampled_pts = tsp_circle[indices]

# Compute IK for each sampled point and add to trajectory for animation
traj = np.empty((0, 6))
for sampled_pt in sampled_pts:
    T_goal = sm.SE3(np.array(sampled_pt)) * sm.SE3(sm.SO3((ee_pose_des_R)))
    q_sol = robot.ikine_LM(T_goal).q
    traj = np.vstack([traj, q_sol])

robot_fig = robot.plot(q_des, block=False, backend='pyplot', dt=dt, jointaxes=False) # turn off quiver for better viz
ax = robot_fig.ax
line, = ax.plot([], [], [], 'r-', linewidth=LW, label="actual")
ax.plot(tsp_circle[:, 0], tsp_circle[:, 1], tsp_circle[:, 2], 'b--', linewidth=LW, label="desired") # desired ee_traj

for i, q in enumerate(traj):
    robot.q = q  # set the robot configuration
    robot_fig.step()
    line.set_data(sampled_pts[:i+1, 0], sampled_pts[:i+1, 1])
    line.set_3d_properties(sampled_pts[:i+1, 2])
    plt.pause(0.01)  
    ax.legend()