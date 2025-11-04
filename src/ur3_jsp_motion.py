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