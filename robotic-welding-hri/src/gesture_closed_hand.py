import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")

import time
import Leap
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
import json


def read_config_file():
    with open("config.json") as json_file:
        config = json.load(json_file)
    return config


def main():
    controller = Leap.Controller()
    controller.config.set("tracking_processing_auto_flip", False)
    controller.config.save()
    closed_hand = False
    closed_hand_time = 0
    last_config_poll_time = 0
    follow_hand_mode = read_config_file()["follow_hand_mode"]
    while 1:
        if time.time() - last_config_poll_time > 1:
            last_config_poll_time = time.time()
            follow_hand_mode = read_config_file()["follow_hand_mode"]

        if follow_hand_mode != "scanning":
            time.sleep(1.1)
            print("Not scanning!")

            continue
        frame = controller.frame()
        if len(frame.hands):
            extended_finger_list = frame.fingers.extended()
            number_extended_fingers = len(extended_finger_list)
            if number_extended_fingers == 0:
                if closed_hand == False:
                    closed_hand_time = time.time()
                closed_hand = True
                if closed_hand == True and time.time() - closed_hand_time > 1:
                    print("Closed Hand")
                    config = read_config_file()
                    config["prev_follow_hand_mode"] = "scanning"
                    config["follow_hand_mode"] = "off"
                    with open("config.json", "w") as json_file:
                        json.dump(config, json_file, indent=4)
                    time.sleep(1.1)
                else:
                    print("Pending Closed Hand")
                # print(index_middle_tip_distance)
            else:
                print("No Closed Hand")
                closed_hand = False
                # print('Number of Extended Fingers: %s; Index: %s, Middle: %s' % (number_extended_fingers,index_extended,middle_extended))
            # print(len(extended_finger_list))


# if __name__ == "__main__":
main()