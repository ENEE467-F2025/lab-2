import roboticstoolbox as rtb 
import numpy as np # library containing important kernels for matrix math

# Load a UR3 model
robot = rtb.models.DH.UR3() # UR3 class instance

# Define a joint configuration (6 values; one for each joint)
q = np.array([0, -np.pi/4, np.pi/4, 0, np.pi/3, 0])

# Forward kinematics: joint space --> task space
T = robot.fkine(q)
