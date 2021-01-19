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


#     print("Going to start")
#     # startj = [-1.0369790236102503, -1.0823825041400355, -2.211877171193258, 0.15503239631652832, 1.0424913167953491, -0.0012467543231409195]
#     # rob.movej(startj, acc=0.8, vel=0.4, wait=True)
    initj = rob.getj() #result in radius
    print("Initial joint configuration is ", initj)
#     # t = rob.get_pose() #
#     # print("Transformation from base to tcp is: ", t)

#     p1 = [2.1390762329101562, -1.977527920399801, 2.101027011871338, -1.9881075064288538, -1.7569130102740687, 0.526220977306366]
#     p2 = [2.0637872219085693, -1.7856486479388636, 2.19935941696167, -2.290964428578512, -1.7351067701922815, 0.45302730798721313]
# #     p3 = [0.7490283250808716, -2.1367180983172815, 2.29533314704895, -1.9723175207721155, -1.3257330099688929, -0.8057563940631312]

#     vx = -0.01
#     vy = 0
#     vz = 0

#     vrx = 0
#     vry = 0
#     vrz = 0

#     a = 0.05
#     t = 8

#     rob.movej(p1, acc=0.1, vel=0.2, wait=True)
#     rob.movej(p2, acc=0.1, vel=0.2, wait=True)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
    
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
    
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, 1)
    
        
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
    
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, 1)
      
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, 1)
    
        
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, t)
    
#     rob.speedl([vx,vy,vz,vrx,vry,vrz], a, t)
#     rob.speedl([vx+0.04,vy,vz,vrx,vry,vrz], a, 1)


    
#     time.sleep(12)
#     rob.movej(p3, acc=0.1, vel=0.2, wait=True)

#     print("stop robot")
    rob.stop()

#     # --- In the end remember to close all cv windows
#     # cv2.destroyAllWindows()
    rob.close()


if __name__ == '__main__':
    main(sys.argv)
