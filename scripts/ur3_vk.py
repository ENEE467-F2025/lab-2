#!/usr/bin/env python3

"""
Example file for computing velocity kinematics for a 
6 DoF robot arm
"""
import os
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

robot = rtb.models.DH.UR3()