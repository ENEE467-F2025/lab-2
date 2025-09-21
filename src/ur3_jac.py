import roboticstoolbox as rtb
import numpy as np

robot = rtb.models.DH.UR3()


VERY_HIGH_NUM = 1e12
q_regular = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007] # Non-singular configuration 
q_at_lim = robot.qlim[0]  # lower joint limit of robot, 
                                        # \theta = -\pi


# TODO: Replace with the q values in Q9 (i-v)
q_to_test = [-1.860, -2.990, 0.670, -2.000, 1.610, -0.007]  # Example config

def singular(rank) -> str:
    if rank < 6:
        return "singular"
    return "non-singular"

def check_inv(some_J):
    """Tests for singular configurations by computing
    the Jacobian's condition number
    """
    cond_num = np.linalg.cond(some_J)
    if cond_num >= VERY_HIGH_NUM:
        return "not true"
    return "true"
    
# Jacobians
J_reg = robot.jacob0(q=q_regular)
J_at_lim = robot.jacob0(q=q_at_lim)

# TODO: Replace with your implementation; i.e., change the q argument in robot.jacob0()
J_to_test = robot.jacob0(q=[0, 0, 0, 0, 0, 0])

# matrix ranks
rank_J_reg = np.linalg.matrix_rank(J_reg)
rank_J_at_lim = np.linalg.matrix_rank(J_at_lim)

# TODO: Compute rank for J_to_test using np.linalg.matrix_rank()
rank_J_to_test = None

# try to compute the matrix inverse (Method 1)
try:
    J_reg_inv = np.linalg.inv(J_reg)
except Exception as e:
    print(f"Could not compute the inverse of J_reg_inv: {e}")

try:
    J_at_lim_inv = np.linalg.inv(J_at_lim)
except Exception as e:
    print(f"Could not compute the inverse of J_at_lim: {e}")

# TODO: Modify to compute the inverse of J_to_test, i.e., modify the a argument
try:
    J_to_test_inv = np.linalg.inv(a=np.eye(6))
except Exception as e:
    print(f"Could not compute the inverse of J_to_test: {e}")

# check if q is singular using check_inv (Method 2)
exists_J_reg_inv = check_inv(J_reg)
exists_J_at_lim_inv = check_inv(J_at_lim)

# TODO: check if q_to_test is singular using the check_inv method
exists_J_at_test = check_inv(J_to_test)

print(
    f"The rank of J_reg is {rank_J_reg}, so q_reg is {singular(rank_J_reg)}, \n \
    and it is {exists_J_reg_inv} that its inverse exists \n"
)

print(
    f"The rank of J_at_lim is {rank_J_at_lim}, so q_at_lim is {singular(rank_J_at_lim)}, \n\
    and it is {exists_J_at_lim_inv} that its inverse exists"
)

# TODO: print results for J_to_test using above print statement structure
print("")