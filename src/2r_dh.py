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
# Your Code Here
####################### 
# T_12 = None
# T_02 = None
