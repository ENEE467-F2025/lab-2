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
import numpy as np

links = [
    rtb.RevoluteDH(a=0.5, alpha=0),    # joint 1
    rtb.RevoluteDH(a=0.33, alpha=0)    # joint 2
]

twoR = rtb.DHRobot(links, name="2R")

T_01 = np.array([
    [
        np.cos(links[0].theta),
        -np.sin(links[0].theta) * np.cos(links[0].alpha),
        np.sin(links[0].theta) * np.sin(links[0].alpha),
        links[0].a * np.cos(links[0].theta)
    ],
    [
        np.sin(links[0].theta),
        np.cos(links[0].theta) * np.cos(links[0].alpha),
        -np.cos(links[0].theta) * np.sin(links[0].alpha),
        links[0].a * np.sin(links[0].theta)
    ],
    [
        0,
        np.sin(links[0].alpha),
        np.cos(links[0].alpha),
        links[0].d
    ],
    [
        0, 0, 0, 1
    ]
])

print(twoR)

#######################
# TODO: Construct T_12 using Eq. 3
####################### 
# T_12 = None
# T_02 = T_01 @ T_12
