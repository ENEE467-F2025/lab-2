import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from rtde_receive import RTDEReceiveInterface

ROBOT_IP = "192.168.77.22"

# Read-only connection to the UR3e. No motion commands are sent.
rtde_r = RTDEReceiveInterface(ROBOT_IP)
q = np.array(rtde_r.getActualQ())          # measured joint angles (rad)
tcp = np.array(rtde_r.getActualTCPPose())  # measured TCP pose [x, y, z, rx, ry, rz]

print("measured joint angles (rad):", np.round(q, 4))
print("measured TCP pose:", np.round(tcp, 4))

robot = rtb.models.DH.UR3()

# Forward kinematics from the measured joint angles
T_fk = robot.fkine(q)

# Rebuild the measured TCP pose as an SE(3): position and rotation vector
p = tcp[:3]
rvec = tcp[3:]
theta = np.linalg.norm(rvec)
axis = rvec / theta if theta > 1e-9 else np.array([0.0, 0.0, 1.0])
T_meas = sm.SE3(p) * sm.SE3.AngleAxis(theta, axis)

# Compare model-based and measured tool poses
pos_err = np.linalg.norm(T_fk.t - T_meas.t)
print("FK position error (m):", pos_err)

# Inverse kinematics: feed the measured pose back in
sol = robot.ikine_LM(T_meas)
print("IK converged:", sol.success)

T_ik = robot.fkine(sol.q)
print("IK pose residual (m):", np.linalg.norm(T_ik.t - T_meas.t))
print("measured q:", np.round(q, 4))
print("IK q:      ", np.round(sol.q, 4))
