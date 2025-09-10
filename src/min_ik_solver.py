from utils.urdf_parser import Robot
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from numpy.typing import NDArray
from typing import Union, List, Tuple
np.set_printoptions(suppress=True)
# RTB for verification
robot_rtb = rtb.models.UR3()

# Helper functions
def compute_jacobian(q: List[float], end) -> NDArray:
    """Computes the Jacobian matrix for a given configuration."""
    robot_rtb.q = q
    return robot_rtb.jacob0(q, end=end)

def check_inv(jacobian: NDArray, tol:Union[float, None]=1e12) -> bool:
    """Checks whether a configuration is singular by 
    evaluating the condition number of the 
    Jacobian argument
    """
    cond_num = np.linalg.cond(jacobian)
    if cond_num >= tol:
        return False
    else: 
        return True

def singular(rank) -> str:
    if rank < 6:
        return False
    return True


########################################################################
#                 MODIFY HERE for Question 13 (a)
########################################################################
def compute_ik(robot: Robot, 
                x_d: sm.SE3, 
                init_guess: Robot.Configuration, 
                eps: float = 1e-4, 
                gamma: float = 1.0, 
                damp_factor: float = 0.0, 
                method: str = 'jacinv', 
                max_iters: int = 100) -> Tuple[Robot.Configuration, float, int]:
        """
        Iterative IK using Jacobian pseudoinverse or Damped Least Squares.
        """
        config_k = init_guess.copy()

        for i in range(max_iters):

            # grab current EE pose
            T_ee = robot._compute_fk(config=config_k)

            # compute task-space error
            # translation error
            dt = np.asarray(x_d.t) - np.asarray(T_ee.t)

            # rotation error
            dr = robot.compute_tsp_rot_error(x_d, T_ee)
            error = np.hstack((dt, dr)).reshape((-1, 1))  # 6x1 column

            ########################################################################
            #                 MODIFY HERE for Question 13 (a)
            ########################################################################
            # use Equation 14 to check for task-space convergence

            

            # Jacobian at current q (we use rtb's jacob0 since the URDF parser class 
            # jacobian function has been expunged from the class due to
            # other questions dependent on it)
            J = compute_jacobian(config_k.joint_values, end=robot.ee_link._name)
            if J is None:
                raise RuntimeError("Jacobian computation failed")

            # check joint limits before applying update
            if not robot.check_limits(config=config_k):
                config_k = robot.clamp_limits(config_k, scale=0.8)

            # ensure current q is within limits
            if robot.check_limits(config=config_k):

                # ensure current q is non-singular
                if robot.check_inv(J):
                    # choose update law
                    if method == "dls" and damp_factor > 0.0:
                        JJt = J @ J.T
                        J_pinv = J.T @ np.linalg.inv(JJt + (damp_factor**2) * np.eye(JJt.shape[0]))
                    else:
                        ########################################################################
                        #                 MODIFY HERE for Question 13 (a)
                        ########################################################################
                        # compute the Jacobian pseudo inverse using Numpy's pinv
                        J_pinv = None
                else:
                    # fallback to DLS with small damping
                    damp_eps = 1e-3
                    JJt = J @ J.T
                    J_pinv = J.T @ np.linalg.inv(JJt + (damp_eps**2) * np.eye(JJt.shape[0]))
                ########################################################################
                #                 MODIFY HERE for Question 13 (a)
                ########################################################################
                # update iterate using Equation 13
                delta_q = None
                q_next = None
                config_k = Robot.Configuration(robot.actuated_joints, q_next.tolist())

        # if we exit the loop without convergence
        raise RuntimeError("IK did not converge within max iterations")

####################################
################ TEST ##############
####################################
def main():
    xml_file_path_rel = 'urdf/ur3.urdf'
    robot = Robot(desc_fp=xml_file_path_rel)
    MAX_ITERS = 2000
    DAMP_FACTOR = 0.01
    EPS = 1e-4
    GAMMA = 0.9
    q_test = robot.Configuration(
                                joints=robot.actuated_joints,
                                joint_values=[0, -np.pi/4, np.pi/4, 0, np.pi/3, 0]
                            )
    q_0 = robot.Configuration.zeros_for_joints(robot.actuated_joints)

    # example SE3 poses
    se3_examples = [
        ########################################################################
        #                 MODIFY HERE for Question 13 (a)
        ######################################################################## 
    ]

    for i, x_d in enumerate(se3_examples):
        print("--------------------------------------------------")
        print(f"\n\033[92mTest Case {i+1}: SE3 pose = {x_d.t}\033[0m \n")

        # Get min solver results (will error out if the right values are not returned)
        ik_sol, final_ts_err, num_iters = compute_ik(
                                                    robot=robot,
                                                    x_d=x_d,
                                                    init_guess=q_0,
                                                    eps=EPS,
                                                    gamma=GAMMA,
                                                    damp_factor=DAMP_FACTOR,
                                                    method='jacinv',
                                                    max_iters=MAX_ITERS
                                                )

        # Get RTB results
        ik_sol_rtb = robot_rtb.ikine_LM(Tep=x_d, q0=q_0.joint_values, \
                                        end='tool0', tol=1e-4, slimit=2000)

        # Print results
        print(f"IK Solution (min solver): {np.round(ik_sol.joint_values, 3)}, num_iterations: {num_iters}, error: {np.round(final_ts_err, 5)}\n")


        # Check FK for min solver result
        ########################################################################
        #                 MODIFY HERE for Question 13 (c)
        ######################################################################## 

if __name__=='__main__':
    main()