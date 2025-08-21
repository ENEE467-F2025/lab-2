import roboticstoolbox as rtb 
import numpy as np # library containing important kernels for matrix math
import spatialmath as sm # library containing important kernels for tf math

# Load a UR3 model
robot = rtb.models.DH.UR3() # UR3 class instance

# Desired end-effector pose (translation + rotation)
T_goal = sm.SE3(0.5, 0.2, 0.3) * sm.SE3.Rz(np.pi/2)

# Compute IK
solution = robot.ikine_LM(T_goal)   # Levenberg-Marquardt solver
print(solution.q)                   # Joint angles (if solution found)