import sys
sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import urx
import time
import Leap
import math
import numpy as np
import math3d as m3d
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
    required_robot_position = absolute_hand_position + [0, 0.19, 0]
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

    while 1:
        try:
            tool_pose = get_tool_pose(robot)
            T = convert_tool_pose_to_transformation_matrix(tool_pose)
            # print(T)
            frame = controller.frame()
            if len(frame.hands):
                extended_finger_list = frame.fingers.extended()
                number_extended_fingers = len(extended_finger_list)
                if (
                    number_extended_fingers != 5
                ):
                    pass
                else:
                    relative_palm_postion = read_hand_position(frame)
                    absolute_hand_position = calculate_hand_position(T, relative_palm_postion)

                    required_robot_position = calculate_required_robot_position(absolute_hand_position)

                    final_pose = list(required_robot_position)
                    final_pose.extend(tool_pose[3:])

                    pose_difference = np.linalg.norm(np.array(tool_pose[:3]) - np.array(required_robot_position))

                    # Only moves robot if the move is greater than 0.5cm; reduces jitter this way
                    if pose_difference > 0.005:
                        print('\ncurrent_pose: %s' % (tool_pose))
                        print('\nabsolute_hand_position: %s' % (absolute_hand_position))
                        print('required_pose: %s' % (final_pose))
                        print('pose_difference: %s' % (pose_difference))
                        # Only moves robot if move is smaller than 8cm, minimizes robot moving in strange directions
                        if pose_difference < 0.08:
                            # print(T)
                            robot.movep(list(final_pose), acc=0.1, vel=0.2, wait=False)

        except:
            robot.close()
            sys.exit(0)



if __name__ == "__main__":
    main()
