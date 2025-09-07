import roboticstoolbox as rtb
import numpy as np

robot = rtb.models.DH.UR3()

VERY_HIGH_NUM = 1e12
q_regular = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007] # Non-singular configuration
q_at_lim = robot.qlim[0]  # lower joint limit of robot, 
                                        # \theta = -\pi

def check_inv(some_J):
    """Uses condition number as ill-conditioned J
       weirdly passes the np.linalg.LinAlgError check
    """
    cond_num = np.linalg.cond(some_J)
    if cond_num >= VERY_HIGH_NUM:
        return "not true"
    else:
        return "true"
    
# Jacobians
J_reg = robot.jacob0(q=q_regular)
J_at_lim = robot.jacob0(q=q_at_lim)

# matrix ranks
rank_J_reg = np.linalg.matrix_rank(J_reg)
rank_J_at_lim = np.linalg.matrix_rank(J_at_lim)

# matrix inverse
exists_J_reg_inv = check_inv(J_reg)
exists_J_at_lim_inv = check_inv(J_at_lim)

print(
    f"The rank of J_reg is {rank_J_reg}, so q_reg is not singular, \n \
    and it is {exists_J_reg_inv} that its inverse exists \n"
)

print(
    f"The rank of J_at_lim is {rank_J_at_lim}, so q_at_lim is singular, \n\
    and it is {exists_J_at_lim_inv} that its inverse exists"
)