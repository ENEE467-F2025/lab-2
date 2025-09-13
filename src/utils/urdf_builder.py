#!/usr/bin/env python3

"""
Simple class with methods for building robot description files in the Unified Robot Description Format (URDF).

References:
1. K. M. Lynch and F. C. Park, Modern Robotics. Cambridge University Press, 2019.
2. “URDF Primer - MATLAB & Simulink.” [Online]. Available: https://www.mathworks.com/help/sm/ug/urdf-model-import.html
3. “urdf/XML - ROS Wiki.” Available: https://wiki.ros.org/urdf/XML


Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Union
 
class URDFBuilder:
    def __init__(self, name="robot"):
        self.robot = ET.Element("robot", name=name)
        self.links = {}
        self.joints = {}
        self.link_lengths = {}

    def add_link(self, name: str, length:Union[float, None]=None, \
                 radius:Union[float, None]=None, mass:float=1.0, geometry="box"):
        """
        Add a link with a simple geometry.
        """
        link = ET.SubElement(self.robot, "link", name=name)
        
        # simplified inertial
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", value=str(mass))

        # simplified visual
        if length or radius:
            visual = ET.SubElement(link, "visual")
            geometry_tag = ET.SubElement(visual, "geometry")
            if geometry == "box":
                ET.SubElement(geometry_tag, "box", size=f"{length} 0.05 0.05")
            elif geometry == "cylinder":
                ET.SubElement(geometry_tag, "cylinder", length=str(length), radius=str(radius))
        
        self.links[name] = link
        self.link_lengths[name] = length
        return link

    def add_joint(self, name:str, parent:str, child:str, joint_type="revolute",
                  origin=(0,0,0), rpy=(0,0,0), axis=(0,0,1)):
        """
        Add a joint connecting parent to child
        """
        joint = ET.SubElement(self.robot, "joint", name=name, type=joint_type)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin",
                      xyz="{} {} {}".format(*origin),
                      rpy="{} {} {}".format(*rpy))
        if joint_type in ("revolute", "continuous", "prismatic"):
            ET.SubElement(joint, "axis", xyz="{} {} {}".format(*axis))
        
        self.joints[name] = joint
        return joint

    def to_string(self):
        """Pretty-print XML string"""
        rough = ET.tostring(self.robot, "utf-8")
        reparsed = minidom.parseString(rough)
        return reparsed.toprettyxml(indent="  ")

    def save(self, filepath):
        with open(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+'/'+filepath, "w") as f:
            f.write(self.to_string())

    def plot_robot(self, config: List):
        """
        Visualize a planar 2R manipulator
        """
        fig, ax = plt.subplots()
        x, y, theta = 0.0, 0.0, 0.0
        points = [(x, y)]
        if len(self.link_lengths.keys()) > 0:
            for link_name, length in self.link_lengths.items():
                # skip links without a defined length
                if length is None:
                    continue
                try:
                    length = float(length)
                except (TypeError, ValueError):
                    continue
                theta = config[list(self.link_lengths.keys()).index(link_name)]
                x += length * np.cos(theta)
                y += length * np.sin(theta)
                points.append((x, y))
        xs, ys = zip(*points)
        ax.plot(xs, ys, marker='o')
        ax.set_aspect('equal')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title(str(self.robot.attrib['name']) + ' Robot')
        plt.grid(True)
        plt.show()