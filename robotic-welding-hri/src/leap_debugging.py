import sys
sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")

import time
import Leap
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt


def main():
    controller = Leap.Controller()
    controller.config.set("tracking_processing_auto_flip", False);
    controller.config.save();
    while 1:
        frame = controller.frame()
        if len(frame.hands):
            basis = frame.hands[0].basis
            # arr = [basis.x_basis, basis.y_basis,  [0, 0, 1]]
            print(basis.z_basis)
            # palm_pos = list(frame.hands[0].palm_.to_tuple())
            # print(palm_pos)
            # print('\n')
          
            
if __name__ == "__main__":
    main()