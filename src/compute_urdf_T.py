from utils.urdf_parser import Robot
import spatialmath as sm


def print_matrix_colored(matrix):
    """Print a 4x4 matrix with colored formatting for better readability."""
    for i, row in enumerate(matrix):
        row_str = []
        for j, val in enumerate(row):
            if i < 3 and j < 3 and i == j:
                row_str.append('\033[91m' + f"{val: .4f}" + '\033[0m')
            elif j == 3 and i < 3:
                row_str.append('\033[94m' + f"{val: .4f}" + '\033[0m')
            else:
                row_str.append(f"{val: .4f}")
        print(' '.join(row_str))

def main():
    xml_file_path_rel = 'urdf/ur3.urdf'
    robot = Robot(desc_fp=xml_file_path_rel)
    link_names = [link._name for link in robot.links]

    ####################################
    # TODO: Question 2(a)
    ####################################
    q_test = robot.Configuration(
                                joints=robot.actuated_joints,
                                joint_values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
                            )
    
    # list containing frame names in UR3's kinematic chain
    link_list = [
        'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link',
        'wrist_2_link', 'wrist_3_link', 'tool0'
    ]

    # Cache SE3 objects along chain
    SE3_list = []
    for i, link in enumerate(link_list):
        try:
            # fill SE3_list
            SE3_link = robot._compute_T() # modify; fill with link names and modified Configuration object, q_test
            SE3_list.append(SE3_link) 
        except Exception as e:
            print(f"\nCould not compute transformation for {link}: {e}")

    ####################################
    # TODO: Question 5(a)
    # Chain the SE3s in the SE3_list you populated above in 
    # the specified traversal order using matrix multiplication
    ####################################
    SE3_prod_mat = None
    SE3_prod = sm.SE3(SE3_prod_mat)

    ####################################
    # TODO: Question 5(b)
    
    ####################################
    SE3_base_tool0 = None


if __name__=='__main__':
    main()