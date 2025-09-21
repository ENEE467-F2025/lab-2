#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb # import the Toolbox

robot = rtb.models.UR3() # Load a UR3 model

# TODO: Question 5(c)
q = None 

T = robot.fkine(
    q = q,
    end=None 
)

print(T)