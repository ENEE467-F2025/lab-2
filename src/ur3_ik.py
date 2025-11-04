#!/usr/bin/env python3

# Lab 2: Manipulator Kinematics with the Robotics Toolbox for Python
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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