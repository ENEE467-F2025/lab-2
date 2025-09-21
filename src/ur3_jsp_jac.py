#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

robot = rtb.models.DH.UR3() # Load a UR3 model

#######################################################
# TODO: Vary this number. Test 1e3, 1e4, 1e5, and 1e6 ###
VERY_HIGH_NUM = 1e2
#######################################################

# Define the start, mid, and end configurations of the robot arm
q_start = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
q_at_lim = robot.qlim[0]
q_end = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]

def check_inv(some_J):
    """Tests for singular configurations by computing
    the Jacobian's condition number
    """
    cond_num = np.linalg.cond(some_J)
    if cond_num >= VERY_HIGH_NUM:
        return False
    else:
        return True
    
def avoid_singular_q(traj):
    traj_non_sing = np.empty_like(traj[0,:])
    traj_sing = np.empty_like(traj[0,:])
    for q in traj:
        if check_inv(robot.jacob0(q)):
            traj_non_sing = np.vstack([traj_non_sing, q])
        else:
            traj_sing = np.vstack([traj_sing, q])
    return traj_non_sing, traj_sing

# Compute segment trajectories
traj1 = rtb.jtraj(q_start, q_at_lim, 25)   # first 25 steps
traj2 = rtb.jtraj(q_at_lim, q_end, 25)     # last 25 steps

# Concatenate joint trajectories with singular config
q_traj = np.vstack([traj1.q, traj2.q])
q_traj_non_sing, q_traj_sing = avoid_singular_q(q_traj)

# Case 1: Trajectory with singular intermediate configuration
print(f"The naive planner has {q_traj.shape[0]} configs \n")   # q_traj will have 50 steps going through start -> mid -> end
robot.plot(q_traj, block=False) # Animate the motion
plt.close()                      

# Case 2: Singularity-free trajectory
print("-------------------------------------------")
print(f"The singularity-aware planner has {q_traj_non_sing.shape[0]} configs \n")   
print(f"Thus, there are {q_traj.shape[0]- q_traj_non_sing.shape[0]} erring configs \n")
print(f"and it is {robot.qlim[0] in q_traj_sing} that robot.qlim[0] is one of them")
robot.plot(q_traj_non_sing, block=False) # Animate the motion