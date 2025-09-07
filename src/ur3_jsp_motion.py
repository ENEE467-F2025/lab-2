#!/usr/bin/env python3

import numpy as np
import roboticstoolbox as rtb

# Load a UR3 model
robot = rtb.models.DH.UR3()

# Define the start and end configurations of the robot arm
q_start = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
q_end = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]

# Interpolate trajectory (50 steps)
traj = rtb.jtraj(q_start, q_end, 50)

# Animate the motion
robot.plot(traj.q, block=True)