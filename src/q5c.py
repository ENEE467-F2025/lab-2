#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb # import the Toolbox

robot = rtb.models.UR3() # Load a UR3 model

q = None # MODIFY HERE for Question 5(c)

T = robot.fkine(
    q = q,
    end=None # MODIFY HERE for Question 5(c)
)

print(T) # T is a spatialmath.SE3 object