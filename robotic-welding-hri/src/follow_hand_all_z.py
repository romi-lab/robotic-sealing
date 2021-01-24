import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import Leap
import os
import urx
import time
import math
import numpy as np
import math3d as m3d
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=4, suppress=True)


def convert_tool_pose_to_transformation_matrix(tool_pose):
    position_vector = np.array(tool_pose[:3]).reshape((3, 1))
    rotation_vector = np.array(tool_pose[3:])

    rotation_matrix = R.from_rotvec(rotation_vector).as_dcm()

    transformation_matrix = np.append(rotation_matrix, position_vector, axis=1)
    transformation_matrix = np.append(
        transformation_matrix, np.array([[0, 0, 0, 1]]), axis=0
    )
    return transformation_matrix


def calculate_rotation_matrix(rotation_vector):
    return R.from_rotvec(np.array(rotation_vector)).as_dcm()


def calculate_rotation_object(rotation_vector):
    return R.from_rotvec(np.array(rotation_vector))


# Calculates hand position in absolute coordinates
def calculate_hand_position(transformation_matrix, relative_palm_postion):

    # TCP to camera axis orientation && m to mm coversion
    hand_coordinates = (np.array(relative_palm_postion) * [-1, -1, 1]) / 1000

    # Converts to auguemented position vector
    hand_coordinates = np.append(hand_coordinates, [1])

    # Gets absolute matrix by transformation matrix multiplication
    absolute_position = transformation_matrix.dot(hand_coordinates)
    return np.round(absolute_position[:3], 3)


def calculate_required_robot_position(absolute_hand_position, y_offset=0):
    required_robot_position = absolute_hand_position + [0, 0.19, 0]
    # required_robot_position = absolute_hand_position + y_offset
    return required_robot_position


def get_tool_pose(robot):
    cartesian_info = robot.secmon.get_all_data()["CartesianInfo"]
    tool_pose = [
        cartesian_info["X"],
        cartesian_info["Y"],
        cartesian_info["Z"],
        cartesian_info["Rx"],
        cartesian_info["Ry"],
        cartesian_info["Rz"],
    ]
    return tool_pose


def read_hand_position(frame):
    return frame["hand_position"]


def get_hand_basis(frame):
    return frame["basis"]


def get_unique_frame(controller, previous_palm_position):
    frame = controller.frame()
    while frame.hands[0].palm_position.to_tuple() == previous_palm_position:
        frame = controller.frame()
    previous_palm_position = frame.hands[0].palm_position.to_tuple()
    return frame, previous_palm_position


def format_frame(frame):
    formatted_frame = {}

    # Hand Basis
    basis_raw = frame.hands[0].basis
    basis = [
        list(basis_raw.x_basis.to_tuple()),
        list(basis_raw.y_basis.to_tuple()),
        list(basis_raw.z_basis.to_tuple()),
    ]
    formatted_frame["basis"] = basis

    formatted_frame["is_left"] = frame.hands[0].is_left

    formatted_frame["extended_fingers_list"] = frame.fingers.extended()

    formatted_frame["hand_position"] = list(frame.hands[0].palm_position.to_tuple())

    return formatted_frame


def get_frame_list(controller, frame_list):
    if len(frame_list) < 5:
        while len(frame_list) < 5:
            frame, previous_palm_position = get_unique_frame(
                controller, previous_palm_position
            )
            frame_list.append(frame)
    else:
        frame_list.pop(0)

        frame, previous_palm_position = get_unique_frame(
            controller, previous_palm_position
        )
        frame_list.append(frame)

    average_frame = frame_list[-1]
    pass


def smooth_frame_data(frame_list):
    pass


def main():
    # Leap motion setup
    controller = Leap.Controller()
    controller.config.save()

    # UR3 setup
    robot = urx.Robot("192.168.0.2")
    time.sleep(1)

    # Set up tool TCP to leap motion position
    mytcp = m3d.Transform()
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.144
    mytcp.pos.z = 0.05
    robot.set_tcp(mytcp)
    previous_palm_position = ("ahahahaha", "haha")
    while 1:
        try:
            # Object containing all data tracked by Leap Motion Camera
            frame, previous_palm_position = get_unique_frame(
                controller, previous_palm_position
            )
            # Extracts information from frame and stores it in dict
            # makes data averageing and filtering data easier
            formatted_frame = format_frame(frame)

            # A custom function is used to get the tool pose as urx's get_pose() function does not return the
            # orientation as displayed on the pendant
            tool_pose = get_tool_pose(robot)

            # Hand basis vectors:
            # 1: Vector from palm to pinky direction
            # 2: Vector from palm to backhand
            # 3: Vector from palm down to wrist 
            basis = get_hand_basis(formatted_frame)

            # Will not track if left hand or is closed!
            # Prevents robot accidentally detecting the right hand as left
            # TO-DO: Make this more robust
            if (
                formatted_frame['is_left']
                or len(formatted_frame['extended_fingers_list']) != 5
            ):
                print("no_instruction")
            else:
                # Rotation object from base to TCP
                rotation_object = calculate_rotation_object(tool_pose[3:]).inv()
                rotation_object = calculate_rotation_object(tool_pose[3:])

                # Orientation around Y axis for calculating absolute hand pose below is not working!!!
                # This fixes it but breaks the X-axis rotation as a result LOL, will fix soon
                flip_axis = R.from_euler("XYZ", np.array([0, 0, math.pi]))
                hand_basis_flipped = flip_axis.apply(basis)

                # Calculate absolute hand pose with respect to base
                hand_basis_absolute = rotation_object.inv().apply(hand_basis_flipped)

                # Get the wanted pose of the robot by rotating the absolute hand pose  
                # i.e.: the hand pose is looking at the robot, we want the pose of the robot looking back
                wanted_basis = hand_basis_absolute
                wanted_basis[0] = wanted_basis[0] * -1
                wanted_basis[1] = wanted_basis[1] * -1

                # X-orientation of hand pose in previous step is broken
                # We ignore it by setting the top right matrix element to 0
                wanted_basis_z_locked = wanted_basis
                wanted_basis_z_locked[2][0] = 0
                wanted_basis_z_locked[2][1] = 0
                wanted_basis_z_locked[2][2] = -1

                wanted_basis_z_locked[0][2] = 0
                wanted_basis_z_locked[1][2] = 0

                wanted_basis_z_locked[0] = wanted_basis_z_locked[0] / np.linalg.norm(
                    wanted_basis_z_locked[0]
                )
                wanted_basis_z_locked[1] = wanted_basis_z_locked[1] / np.linalg.norm(
                    wanted_basis_z_locked[1]
                )
                wanted_basis_z_locked[2] = wanted_basis_z_locked[2] / np.linalg.norm(
                    wanted_basis_z_locked[2]
                )

                # Converts our wanted basis into a rotation vector the UR robot understands
                wanted_rotation_vector = R.from_dcm(wanted_basis_z_locked).as_rotvec()

                # Gets the absoulte position of hand
                relative_palm_postion = read_hand_position(formatted_frame)
                T = convert_tool_pose_to_transformation_matrix(tool_pose)
                absolute_hand_position = calculate_hand_position(
                    T, relative_palm_postion
                )

                # Gets the wanted robot position at distance from palm
                required_robot_position = (wanted_basis_z_locked[1] * 0.19) + np.array(
                    absolute_hand_position
                )

                # Combines previously calculated robot position and rotation vector into one pose
                final_pose = list(
                    np.append(required_robot_position, wanted_rotation_vector)
                )

                # Moves robot
                robot.movep(final_pose, acc=0.04, vel=0.04, wait=False)
                print(wanted_basis_z_locked)
                print(list(wanted_rotation_vector))
                print("\n--------------------------------------\n")
                # time.sleep(0.3)

        except:
            robot.close()
            os._exit
            sys.exit(0)


if __name__ == "__main__":
    main()

