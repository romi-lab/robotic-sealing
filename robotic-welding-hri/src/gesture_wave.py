import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import math
import time
import Leap
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
import json
plt.style.use('ggplot')


def read_config_file():
    with open("config.json") as json_file:
        config = json.load(json_file)
    return config


def main():
    # Initialize leap motion controller object
    controller = Leap.Controller()
    last_config_poll_time = 0
    follow_hand_mode = read_config_file()["follow_hand_mode"]
    # Breaks loop and stops script when wave is detected
    wave_detected = False
    while 1:
        if time.time() - last_config_poll_time > 1:
            last_config_poll_time = time.time()
            follow_hand_mode = read_config_file()["follow_hand_mode"]

        if follow_hand_mode != "off":
            time.sleep(1.1)
            print("Not off!")
            continue

        # Initialize arrays for storing hand orientation data
        hand_direction_x = []
        hand_direction_y = []
        hand_direction_z = []
        extended_fingers = []

        # Captures hand orientation data in 1 second windows
        #
        # Means you will need to be waving for an entire 1 second window duration for the
        # wave to be detected
        start_time = time.time()
        angle_array = []
        time_array = []
        while time.time() < start_time + 1:
            # Polls controller for frame containing hand/bones/joints data
            frame = controller.frame()
            if len(frame.hands):
                time_array.append(time.time() - start_time)
                # Get the direction of hand in frame
                #### TO-DO: Deal with 2 hands in frame ####
                hand_direction = frame.hands[0].direction
                print(hand_direction)
                # Store hand orientations; note Z is inverted
                hand_direction_x.append(hand_direction[0])
                hand_direction_y.append(hand_direction[1])
                hand_direction_z.append(-1 * hand_direction[2])
                extended_fingers.append(len(frame.fingers.extended()))

        # Hard-coded angle calculation from captured hand orientation vectors
        #### TO-DO: Clean this up! :)
        for i in range(len(hand_direction_x)):
            angle = 0
            if hand_direction_x[i] > 0:
                angle = math.atan(hand_direction_x[i] / hand_direction_z[i])
                if angle < 0:
                    angle = 3.14159265359 - (-1 * angle)
            else:
                angle = 1 * math.atan(hand_direction_x[i] / hand_direction_z[i])
                if angle > 0:
                    angle = -1 * (3.14159265359 - (1 * angle))
            angle_array.append(angle)

        if len(angle_array) == 0:
            continue

        initial_angle = angle_array[0]
        angular_difference_array = []
        time_array_duplicates_removed = []
        for i in range(len(angle_array)):
            if i == len(angle_array) - 1:
                time_array_duplicates_removed.append(time_array[i])
                break
            if angle_array[i] - angle_array[i+1] == 0:
                continue
            if angle_array[i] - angle_array[i+1] > 3.14159265359:
                angular_difference_array.append(-6.28318531 + (angle_array[i] - angle_array[i+1]))
            elif angle_array[i] - angle_array[i+1] < -3.14159265359:
                angular_difference_array.append(6.28318531 + (angle_array[i] - angle_array[i+1]))
            else:
                angular_difference_array.append(angle_array[i] - angle_array[i+1])
            time_array_duplicates_removed.append(time_array[i])


        corrected_angle_array = [0]

        for i in angular_difference_array:
            corrected_angle_array.append(corrected_angle_array[-1] + i)

        # Convert to np array
        corrected_angle_array = np.array(corrected_angle_array)
        angular_difference_array = np.array(angular_difference_array)
    
        # Smooth angles data with moving average filter with window size: *19*
        # if len(corrected_angle_array) > 0:
        #     corrected_angle_array = np.convolve(corrected_angle_array, np.ones(5), "valid") / 5
        # if len(time_array) > 0:
        #     time_array_duplicates_removed = np.convolve(time_array_duplicates_removed, np.ones(5), "valid") / 5

        # Set divider between peak and trough detection values to mean max&min angles
        peak_base = (
            ((max(corrected_angle_array) + min(corrected_angle_array)) / 2) if len(corrected_angle_array) > 0 else 0
        )

        # Calculate peaks and troughs angle values
        peaks, _ = find_peaks(corrected_angle_array, height=peak_base)
        troughs, _ = find_peaks(-corrected_angle_array, height=peak_base)

        # wave_gesture_amplitude is calulated by difference between average angles of peaks and troughs
        average_peaks = np.average(corrected_angle_array[peaks])
        average_troughs = np.average(corrected_angle_array[troughs])
        wave_gesture_amplitude = abs(average_peaks - average_troughs)

        # wave count is calucalted by average number of peaks and troughs
        wave_count = (len(peaks) + len(troughs)) / 2
        # print('Average Count Peaks & Troughs:   %s!' % (wave_count))

        average_extended_fingers = np.mean(extended_fingers)
        # Wave is detected if the wave gesture amplitude and wave count exceed a set threshold
        wave_detected = (
            True
            if wave_gesture_amplitude > 0.55
            and wave_count >= 3
            and average_extended_fingers >= 4.4
            else False
        )
        if wave_detected:
            print("\nHand wave detected! :) \n")
            print(len(time_array))
            print(len(angle_array))
            print(len(angular_difference_array))
            print(len(corrected_angle_array))
            config = read_config_file()
            config['prev_follow_hand_mode'] = 'off'
            config["written_to_excel"] = False
            config['follow_hand_mode'] = 'positioning'
            with open("config.json", "w") as json_file:
                json.dump(config, json_file, indent=4)
            time.sleep(1.1)
            # print(time_array)
            plt.plot(time_array_duplicates_removed,corrected_angle_array, "r-")
            # plt.plot(peaks, corrected_angle_array[peaks], "x")
            # plt.plot(troughs, corrected_angle_array[troughs], "o")
            plt.ylabel('Angle Difference from First Measurement (Rad)')
            plt.xlabel('Time (s)')
            # plt.show()
        else:
            print("No hand wave detected :(")
        

# if __name__ == "__main__":
main()
