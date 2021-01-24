import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")

import time
import Leap
import json
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt


def read_config_file():
    with open("config.json") as json_file:
        config = json.load(json_file)
    return config


def main():
    controller = Leap.Controller()
    controller.config.set("tracking_processing_auto_flip", False)
    controller.config.save()
    ok_signed = False
    ok_signed_time = 0
    last_config_poll_time = 0
    follow_hand_mode = read_config_file()["follow_hand_mode"]
    while 1:
        if time.time() - last_config_poll_time > 1:
            last_config_poll_time = time.time()
            follow_hand_mode = read_config_file()["follow_hand_mode"]

        if follow_hand_mode != "2dof":
            time.sleep(1.1)
            print("Not 2dof!")
            continue

        frame = controller.frame()
        if len(frame.hands):
            extended_finger_list = frame.fingers.extended()
            thumb_extended = False
            index_extended = False
            middle_extended = False
            ring_extended = False
            pinky_extended = False
            number_extended_fingers = len(extended_finger_list)

            # Calculate distance from index to thumb
            index_finger_tip_pos = (
                frame.hands[0]
                .fingers.finger_type(Leap.Finger.TYPE_INDEX)[0]
                .bone(3)
                .next_joint.to_tuple()
            )
            thumb_finger_tip_pos = (
                frame.hands[0]
                .fingers.finger_type(Leap.Finger.TYPE_THUMB)[0]
                .bone(3)
                .next_joint.to_tuple()
            )
            index_thumb_tip_distance = np.linalg.norm(
                np.array(index_finger_tip_pos) - np.array(thumb_finger_tip_pos)
            )

            # Check extendedness of fingers
            for finger in extended_finger_list:
                if finger.type == Leap.Finger.TYPE_THUMB:
                    thumb_extended = True
                if finger.type == Leap.Finger.TYPE_INDEX:
                    index_extended = True
                if finger.type == Leap.Finger.TYPE_MIDDLE:
                    middle_extended = True
                if finger.type == Leap.Finger.TYPE_RING:
                    ring_extended = True
                if finger.type == Leap.Finger.TYPE_PINKY:
                    pinky_extended = True

            if (
                number_extended_fingers == 3
                and thumb_extended == False
                and index_extended == False
                and middle_extended == True
                and ring_extended == True
                and pinky_extended == True
                and index_thumb_tip_distance < 30
            ):
                if ok_signed == False:
                    ok_signed_time = time.time()
                ok_signed = True
                if ok_signed == True and time.time() - ok_signed_time > 1:
                    print("OK Sign!!!")
                    config = read_config_file()
                    config['prev_follow_hand_mode'] = '2dof'
                    config['follow_hand_mode'] = 'scanning'
                    with open("config.json", "w") as json_file:
                        json.dump(config, json_file, indent=4)
                    time.sleep(1.1)
                else:
                    print("Pending ok sign :0")
                # print(index_thumb_tip_distance)
            else:
                print("No OK sign yet :(((")
                print(index_thumb_tip_distance)
                ok_signed = False
                # print('Number of Extended Fingers: %s; Index: %s, Middle: %s' % (number_extended_fingers,index_extended,middle_extended))
            # print(len(extended_finger_list))


# if __name__ == "__main__":
main()