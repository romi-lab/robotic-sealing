import sys
sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import urx
import time
import Leap
import math
import numpy as np
import math3d as m3d
import copy
from scipy.spatial.transform import Rotation as R

# Converts URx's rotation vector into a rotation matrix
#
# I did not derive this nor do I fully understand the maths behind this :0
# I took it from: https://dof.robotiq.com/discussion/1648/around-which-axes-are-the-rotation-vector-angles-defined
def convert_tool_pose_to_transformation_matrix(tool_pose):
    position_vector = np.array(tool_pose[:3]).reshape((3,1))
    rotation_vector = np.array(tool_pose[3:])
    
    rotation_matrix = R.from_rotvec(rotation_vector).as_dcm()

    transformation_matrix = np.append(rotation_matrix, position_vector, axis = 1)
    transformation_matrix = np.append(transformation_matrix ,np.array([[0,0,0,1]]), axis=0)
    return transformation_matrix

def calculate_coordinate_system_from_hand():
    pass

def convert_coordinate_spaces_to_rotation_vector(A_prime):
    A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    rotation_matrix = np.dot(A, np.linalg.inv(A_prime))
    r = R.from_matrix(rotation_matrix)
    return r.as_rotvec

# Calculates hand position in absolute coordinates
def calculate_hand_position(transformation_matrix, relative_palm_postion):
    # Formats raw hand coordinates
    hand_coordinates_raw = relative_palm_postion
    # m to mm converstion
    hand_coordinates = (np.array(hand_coordinates_raw) * [-1, -1, 1]) / 1000
    
    # Converts to auguemented position vector 
    hand_coordinates = np.append(hand_coordinates, [1])

    # Gets abolsolute matrix by transformation matrix multiplication
    absolute_position = transformation_matrix.dot(hand_coordinates)
    return np.round(absolute_position[:3], 3)


def calculate_required_robot_position(absolute_hand_position, y_offset=0):
    required_robot_position = absolute_hand_position + [0, 0.19, 0.]
    # required_robot_position = absolute_hand_position + y_offset
    return required_robot_position

def get_tool_pose(robot):
    cartesian_info = robot.secmon.get_all_data()['CartesianInfo']
    tool_pose = [cartesian_info['X'],cartesian_info['Y'],cartesian_info['Z'],cartesian_info['Rx'],cartesian_info['Ry'],cartesian_info['Rz']]
    # tool_pose = [50, -600, -135, 0, 3.14, 0]
    return tool_pose

def read_hand_position(frame):
    return list(frame.hands[0].palm_position.to_tuple())


def main():
    # Leap motion 
    controller = Leap.Controller()
    controller.config.save()

    # UR3 robot config
    robot = urx.Robot("192.168.0.2")
    mytcp = m3d.Transform()  # create a matrix for our tool tcp
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.144
    mytcp.pos.z = 0.05
    time.sleep(1)
    robot.set_tcp(mytcp)
    time.sleep(1)
    counter = 0
    while 1:
        try:
            if counter < 4:
                counter += 1
                time.sleep(0.1)
                continue
            # Calculate pose of robot
            # tool_pose = [0.115, -0.65, 0.432, 0, 3.14, 0]
            tool_pose = get_tool_pose(robot)
            T = convert_tool_pose_to_transformation_matrix(tool_pose)
            frame = controller.frame()
            # Calculate relative position of hand
            relative_palm_postion = read_hand_position(frame)
            # Calculate absolute position of hand via robot pose
            absolute_hand_position = calculate_hand_position(T, relative_palm_postion)
            # Calculate vector of robot to hand [V_hr]
            # print(absolute_hand_position)
            robot_to_hand_vector = np.array(tool_pose[:3]) - absolute_hand_position 
            # print(robot_to_hand_vector)
            # Normalize vector
            robot_to_hand_vector = robot_to_hand_vector / np.linalg.norm(robot_to_hand_vector)
            # Set wanted basis:
            wanted_basis = [[],[],[]]
            # X-axis: Normal to ground plane; Z-componenet zeroed
            # print(robot_to_hand_vector)
            wanted_basis[0] = copy.copy(robot_to_hand_vector)
            wanted_basis[0][2] = 0
            wanted_basis_x = list(wanted_basis[0])
            wanted_basis_x[0], wanted_basis_x[1] = (-1 * wanted_basis_x[1]),  wanted_basis_x[0]
            wanted_basis[0] = wanted_basis_x
            wanted_basis[0] = wanted_basis[0] / np.linalg.norm(wanted_basis[0])
            # print(robot_to_hand_vector)
            # Y axis: V_hr
            wanted_basis[1] = robot_to_hand_vector
            # Z-axis: Calculated via. RH rule
            wanted_basis[2] = np.cross(wanted_basis[0], wanted_basis[1])
            print(np.array(wanted_basis))
            # Convert wanted basis to rotation matrix and then rotation vector
            wanted_rotation_vector = R.from_dcm(wanted_basis).as_rotvec()
            # Send the updated rotation vector to robot with fixed position
            final_pose = list(
                np.append(np.array(tool_pose[:3]), wanted_rotation_vector)
            )
            robot.movel(final_pose, acc=0.4, vel=0.4, wait=True)
            # time.sleep(0.1)
            print(final_pose)
            print('\n\n')
        # time.sleep(0.4)
        except:
            robot.close()
            sys.exit(0)



if __name__ == "__main__":
    main()
