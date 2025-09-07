#!/usr/bin/env python3
from utils.urdf_builder import URDFBuilder

####################################
# MODIFY HERE
####################################

# Create a 2R manipulator
builder = URDFBuilder(name="")

# Links
builder.add_link("", length=0.0, geometry="")
builder.add_link("", length=0.0, geometry="")
builder.add_link("")

# Joints
builder.add_joint("", parent="", child="",
                  joint_type="", origin=(), axis=())
builder.add_joint("", parent="", child="",
                  joint_type="", origin=(), axis=())
builder.add_joint("", parent="", child="",
                  joint_type="", origin=())

# Save URDF
print(builder.to_string())
builder.save("")

# Plot robot
builder.plot_robot([])

####################################
# MODIFY HERE
####################################
