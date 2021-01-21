#!/usr/bin/env python
# -*- coding: utf-8 -*-


# from __future__ import division
# import copy
# import math
# from matplotlib import pyplot as plt
# from scipy import signal, ndimage
import sys
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge, CvBridgeError 
# import time
import urx
# import logging



def main(args):

#     logging.basicConfig(level=logging.INFO)

    rob = urx.Robot("192.168.0.2")
#     # rob = urx.Robot("localhost")
    # rob.set_tcp((0, 0.048, 0.227, 0, 0, 0))
#     rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(2, [0, 0, 0])
    tcp_torch = [-0.0002, -0.09216, 0.32202, 0, 0, 0]
    rob.set_tcp(tcp_torch)


    print("Going to start")
    startj = [-1.5366695562945765, -1.6485264937030237, -1.1686652342425745, -1.7946108023272913, 1.4708768129348755, -1.4940932432757776]
    # rob.movej(startj, acc=0.8, vel=0.4, wait=True)
    initj = rob.getj() #result in radius
    print("Initial joint configuration is ", initj)
#     # t = rob.get_pose() #
#     # print("Transformation from base to tcp is: ", t)
    count = 0

    # p2 = [-1.2272198835956019, -1.7004426161395472, -1.6823189894305628, -1.2730239073382776, 1.507084846496582, -1.12502366701235]
    # # rob.movej(p2, acc=0.1, vel=0.2, wait=True)
    # initl = rob.getl() #result in radius
    # print("Initial joint configuration is ", initl)

    # while count < 50:

    #     p1 = [-1.2291749159442347, -1.6868732611285608, -1.5637076536761683, -1.4049976507769983, 1.506557583808899, -1.1270230452166956]
    #     p2 = [-1.2272198835956019, -1.7004426161395472, -1.6823189894305628, -1.2730239073382776, 1.507084846496582, -1.12502366701235]
    #     p3 =  [-0.10069827608962446, -0.5754213343267545, 0.012273970575881643, 2.08606199890458, -2.329059803372046, -0.017347841495701864]

    #     rob.movej(p1, acc=0.1, vel=0.2, wait=True)
    #     rob.movej(p2, acc=0.1, vel=0.2, wait=True)
    #     rob.movel(p3, acc=0.02, vel=0.002, wait=True)

    #     rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)
    # #     time.sleep(12)
    #     rob.movej(p1, acc=0.1, vel=0.2, wait=True)

    #     count = count + 1
    #     print count

    print("stop robot")
    rob.stop()

#     # --- In the end remember to close all cv windows
#     # cv2.destroyAllWindows()
    rob.close()


if __name__ == '__main__':
    main(sys.argv)
