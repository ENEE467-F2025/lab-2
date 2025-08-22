import roboticstoolbox as rtb 
import numpy as np # library containing important kernels for matrix math

robot = rtb.models.DH.UR3() # # Load a UR3 model
q = np.array([0, -np.pi/4, np.pi/4, 0, np.pi/3, 0]) # joint configuration 
T = robot.fkine(q) # Compute FK