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

from utils.urdf_parser import Robot # custom URDF parser with FK kernels
import numpy as np
import spatialmath as sm
from numpy.typing import NDArray
import roboticstoolbox as rtb



def compute_jacobian(robot: Robot, config: Robot.Configuration) -> NDArray:
        """
        Computes the chain's (space) Jacobian at the specified configuration.
        """
        config_dict = config.joint_dict
        T_ee: sm.SE3 = robot._compute_fk(config=config)
        n = len(config_dict.values())
        J = np.zeros((6, n))  # start with a Numpy array of zeros

        for i, joint in enumerate(robot.actuated_joints):
            # get transform to the joint's child frame
            T_0_i_prev = robot._compute_fk(end=joint.child._name, config=config) # link i-1 is the child of joint i
            z_i_prev = T_0_i_prev.R @ np.array(joint.axis) 

            ####################################
            # TODO: Fill in p_i_prev and p_ee
            ####################################

            p_i_prev = None
            p_ee = None

            ####################################
            # TODO: Compute Jv and Jw for the 
            # revolute joint
            ####################################

            if joint.joint_type == "revolute":
                Jv = None
                Jw = None

            elif joint.joint_type == "prismatic":
                Jv = z_i_prev
                Jw = np.zeros(3)
            else:  # fixed joint
                Jv = np.zeros(3)
                Jw = np.zeros(3)

            # Fill Jacobian
            J[:3, i] = Jv
            J[3:, i] = Jw

        return J






####################################
# TEST
####################################
q_arr = [0, -np.pi/4, np.pi/4, 0, np.pi/3, 0]

def main():
    xml_file_path_rel = 'urdf/ur3.urdf'
    robot = Robot(desc_fp=xml_file_path_rel)
    q_test = robot.Configuration(
                                joints=robot.actuated_joints,
                                joint_values=q_arr
                            )
    jac = compute_jacobian(robot=robot, config=q_test)

    print("\n--------------------------")
    print(f"Simple URDF Jacobian Calculator: \n{np.round(jac, 4)}\n")
    # TODO: Add print commands for verification with RTB


if __name__=='__main__':
    main()