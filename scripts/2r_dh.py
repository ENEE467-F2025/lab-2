import roboticstoolbox as rtb

# Standard DH parameters:
# [theta, d, a, alpha]
# q1 and q2 are the joint variables

links = [
    rtb.RevoluteDH(a=0.5, alpha=0),   # Joint 1
    rtb.RevoluteDH(a=0.5, alpha=0)    # Joint 2
]

twoR = rtb.DHRobot(links, name="2R")
print(twoR)
