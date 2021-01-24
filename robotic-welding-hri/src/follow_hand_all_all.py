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
import traceback
import json
import pandas as pd

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
    # A custom function is used to get the tool pose as urx's get_pose() function does not return the
    # orientation as displayed on the pendant
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
    # Hand basis vectors:
    # 1: Vector from palm to pinky direction
    # 2: Vector from palm to backhand
    # 3: Vector from palm down to wrist
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


def get_frame_list(controller, frame_list, previous_palm_position):
    if len(frame_list) < 3:
        while len(frame_list) < 3:
            frame, previous_palm_position = get_unique_frame(
                controller, previous_palm_position
            )
            formatted_frame = format_frame(frame)
            frame_list.append(formatted_frame)
    else:
        frame_list.pop(0)

        frame, previous_palm_position = get_unique_frame(
            controller, previous_palm_position
        )
        formatted_frame = format_frame(frame)
        frame_list.append(formatted_frame)

    # average_frame = frame_list[-1]
    return frame_list


def average_frame_list(frame_list):
    average_frame = frame_list[-1]
    average_basis = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    average_position = [0, 0, 0]
    # Sum all calues across frames
    for frame in frame_list:
        for i in range(len(average_position)):
            average_position[i] += frame["hand_position"][i]
        for i in range(len(frame["basis"])):
            for j in range(len(average_basis[i])):
                average_basis[i][j] += frame["basis"][i][j]

    # Average values by dividing by count
    average_basis = [
        [vector_component / len(frame_list) for vector_component in vector]
        for vector in average_basis
    ]

    # Edit the returned array
    average_position = [i / len(frame_list) for i in average_position]
    average_frame["basis"] = average_basis
    average_frame["position"] = average_position
    return average_frame


def read_config_file():
    with open("config.json") as json_file:
        config = json.load(json_file)
    return config


def get_basis_2dof(wanted_basis):
    # X-orientation of hand pose in previous step is broken
    # We ignore it by setting the top right matrix element to 0
    wanted_basis_final = wanted_basis
    wanted_basis_final[0][2] = 0
    wanted_basis_final[0] = wanted_basis_final[0] / np.linalg.norm(
        wanted_basis_final[0]
    )

    # For some reason X-axis orientation is not ignored??
    # but the desired orientation is mirrored :0
    # Setting the X component of the Z basis to negative fixes this for some reason?
    # I don't know why, will investigate
    wanted_basis_final[2][0] = wanted_basis_final[2][0] * -1
    return wanted_basis_final


def get_basis_positioning(wanted_basis):
    # X-orientation of hand pose in previous step is broken
    # We ignore it by setting the top right matrix element to 0
    wanted_basis_final = wanted_basis
    wanted_basis_final[2][0] = 0
    wanted_basis_final[2][1] = 0
    wanted_basis_final[2][2] = -1

    wanted_basis_final[0][2] = 0
    wanted_basis_final[1][2] = 0

    wanted_basis_final[0] = wanted_basis_final[0] / np.linalg.norm(
        wanted_basis_final[0]
    )
    wanted_basis_final[1] = wanted_basis_final[1] / np.linalg.norm(
        wanted_basis_final[1]
    )
    wanted_basis_final[2] = wanted_basis_final[2] / np.linalg.norm(
        wanted_basis_final[2]
    )
    return wanted_basis


def main():
    # Leap motion setup
    controller = Leap.Controller()
    controller.config.save()

    # UR3 setup
    robot = urx.Robot("192.168.0.2")
    time.sleep(1)

    # Set up tool TCP to sealer position
    # Will result in wrong hand position but correct sealer position, no effect on usage
    mytcp = m3d.Transform()
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.09302
    mytcp.pos.z = 0.32591
    robot.set_tcp(mytcp)
    frame_list = []
    previous_palm_position = ("ahahahaha", "haha")
    config = read_config_file()
    last_config_poll_time = 0
    last_data_time = 0
    tool_position_time_series = []
    while 1:
        if time.time() - last_config_poll_time > 1:
            last_config_poll_time = time.time()
            config = read_config_file()

        if config["follow_hand_mode"] == "off":
            if (
                    config["prev_follow_hand_mode"] == "scanning"
                    and config["written_to_excel"] == False
                ):
                    df = pd.DataFrame(
                        tool_position_time_series,
                        columns=["X", "Y", "Z", "Rx", "Ry", "Rz", "time"],
                    )
                    df.to_excel("tracking_data.xlsx")
                    config["written_to_excel"] = True
                    with open("config.json", "w") as json_file:
                        json.dump(config, json_file, indent=4)
            time.sleep(1.1)
            continue
        try:
            # Object containing all data tracked by Leap Motion Camera

            # Get frame list
            frame_list = get_frame_list(controller, frame_list, previous_palm_position)
            formatted_frame = average_frame_list(frame_list)
            # Extracts information from frame and stores it in dict
            # makes data averageing and filtering data easier
            # formatted_frame = format_frame(frame)

            tool_pose = get_tool_pose(robot)

            basis = get_hand_basis(formatted_frame)
            print(basis)

            # Will not track if left hand or is closed!
            # Prevents robot accidentally detecting the right hand as left
            # TO-DO: Make this more robust
            if (
                formatted_frame["is_left"]
                or len(formatted_frame["extended_fingers_list"]) != 5
            ):
                if formatted_frame["is_left"]:
                    print("Not Following left hand :0")
                if len(formatted_frame["extended_fingers_list"]) != 5:
                    print("Fully open your hand to track :)")
            else:
                # Rotation object from base to TCP
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

                # Here we determine which kind of hand-tracking mode we are to use:
                # i.e.: robot positioning (yaw only) or groove 2dof (yaw and roll)
                if config["follow_hand_mode"] == "2dof":
                    wanted_basis_final = get_basis_2dof(wanted_basis)
                elif config["follow_hand_mode"] == "scanning":
                    wanted_basis_final = get_basis_2dof(wanted_basis)
                elif config["follow_hand_mode"] == "positioning":
                    wanted_basis_final = get_basis_positioning(wanted_basis)

                # Converts our wanted basis into a rotation vector the UR robot understands
                wanted_rotation_vector = R.from_dcm(wanted_basis_final).as_rotvec()

                # Gets the absoulte position of hand
                relative_palm_postion = read_hand_position(formatted_frame)
                T = convert_tool_pose_to_transformation_matrix(tool_pose)
                absolute_hand_position = calculate_hand_position(
                    T, relative_palm_postion
                )

                # Gets the wanted robot position at distance from palm
                required_robot_position = (wanted_basis_final[1] * 0.19) + np.array(
                    absolute_hand_position
                )

                # Combines previously calculated robot position and rotation vector into one pose
                final_pose = list(
                    np.append(required_robot_position, wanted_rotation_vector)
                )

                rotation_difference = np.dot(
                    rotation_object.as_dcm().T,
                    R.from_rotvec(wanted_rotation_vector).as_dcm(),
                )

                angular_difference = np.rad2deg(
                    np.arccos((np.trace(rotation_difference) - 1) / 2)
                )

                position_difference = np.linalg.norm(
                    np.array(tool_pose[:3]) - np.array(required_robot_position)
                )
                if config["follow_hand_mode"] == "scanning":
                    if  time.time() - last_data_time > 0.2:
                        last_data_time = time.time()
                        tool_pose.append(time.time())
                        tool_position_time_series.append(tool_pose)
                print("Tool position: %s" % (tool_pose[:3]))
                print("angular difference: %s" % (angular_difference))
                print("position difference: %s" % (position_difference))
                if angular_difference > 8 or position_difference > 0.025:
                    robot.movep(final_pose, acc=0.06, vel=0.06, wait=False)
                else:
                    print(
                        "position/angular difference not big enough, robot not moved!"
                    )
                print(wanted_basis_final)
                print(list(wanted_rotation_vector))
                print("\n--------------------------------------\n")

        # except Exception:
        except:
            # print(traceback.format_exc())
            robot.close()
            os._exit
            sys.exit(0)


if __name__ == "__main__":
    main()
