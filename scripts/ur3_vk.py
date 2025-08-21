#!/usr/bin/env python3

import os
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

robot = rtb.models.DH.UR3() # Load UR3 model

q = [0.5, -1.0, 0.5, -1.2, 1.0, 0.2] # Joint configuration

J = robot.jacob0(q) # Jacobian

qd = np.array([0.1, 0.0, -0.2, 0.0, 0.1, 0.0]) # Joint velocities rad/s

xdot = J @ qd # 6D end-effector twist

print("End-effector velocity:\n", xdot)
