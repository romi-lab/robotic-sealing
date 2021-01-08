#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import copy
import math
from matplotlib import pyplot as plt
from scipy import signal, ndimage
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose
from cv_bridge import CvBridge, CvBridgeError 
import time
import urx
import logging

def wait():
    raw_input("please enter to continue: ")

# --- Define our Class
class motion_controller:

    def __init__(self):

        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # publish processed img for visualization
        # self.image_pub = rospy.Publisher("rs_rgb_image", Image, queue_size=1)        
        # self.pre_roi_pub = rospy.Publisher("l", Point, queue_size=1)
        # self.pre_roi_info = Point()
        # self.bridge = CvBridge()

        # subscribe to get the info for feedback control
        # pre roi information, includes: k, b, and d
        rospy.Subscriber("logi_info/pre_roi", Point, self.callback_logi_info_pre_roi, queue_size=1, buff_size=52428800)
        # post roi information, includes: w, h, and d
        rospy.Subscriber("logi_info/post_roi", Point, self.callback_logi_info_post_roi, queue_size=1, buff_size=52428800)
        # pre roi information, includes: k, b, and d
        rospy.Subscriber("rs_info/rgb/pre_roi", Point, self.callback_rs_info_rgb_pre_roi, queue_size=1, buff_size=52428800)

    def callback_logi_info_pre_roi(self, Point):

        global logi_pre
        
        logi_pre_k = Point.x
        logi_pre_b = Point.y
        logi_pre_d = Point.z

        logi_pre = [logi_pre_k, logi_pre_b, logi_pre_d]
    
    def callback_logi_info_post_roi(self, Point):

        global logi_post
        
        logi_post_w = Point.x
        logi_post_h = Point.y
        logi_post_d = Point.z

        logi_post = [logi_post_w, logi_post_h, logi_post_d]
  
    def callback_rs_info_rgb_pre_roi(self, Point):

        global rs_pre
        
        logi_pre_k = Point.x
        logi_pre_b = Point.y
        logi_pre_d = Point.z
       
        rs_pre = [logi_pre_k, logi_pre_b, logi_pre_d]
    

def seam_tracking(rob):
 
    global logi_pre, logi_post, rs_pre

    start = time.time()

    while True:

        vx = -0.01
        vy = 0
        vz = 0

        vrx = 0
        vry = 0
        vrz = 0

        a = 0.001
        t = 1

        if logi_pre and rs_pre:

            y_diff = rs_pre[-1]
            z_diff = logi_pre[-1]
            y_scale = 0.001
            z_scale = 0.001

            if not -20 <= y_diff <= -4 :
                vy = y_diff*y_scale
                print "changing in y"
                if abs(vy) > 0.002:
                    print "in if"
                    if vy>0:
                        vy=0.002
                    else:
                        vy=-0.002
            # else:
            #     vy = 0
            #     if abs(z_diff) > 20:
            #         vz = z_diff*z_scale
            #         print "changing in z"
            #         if abs(vy) > 0.01:
            #             print "in if"
            #             if vz>0:
            #                 vz=0.001
            #             else:
            #                 vz=-0.001
            #     else:
            #         vz = 0
            #         print "all well"
            
        vel_speed = [vx,vy,vz,vrx,vry,vrz]
        print "moving with speed" + str(vel_speed)
        rob.speedl(vel_speed, a, t)

        end = time.time()
        print int(end-start) 

        if end-start > 50:
            rob.stop()
            break



# --------------- MAIN LOOP
def main(args):


    global logi_pre, logi_post, rs_pre
    logi_pre = []
    logi_post = []
    rs_pre = []

    logging.basicConfig(level=logging.INFO)

    rob = urx.Robot("192.168.0.2")
    # rob = urx.Robot("localhost")
    # rob.set_tcp((0, 0.048, 0.227, 0, 0, 0))
    # rob.set_tcp((0, 0, 0, 0, 0, 0))
    # rob.set_payload(1.2, [0, 0.03, 0.03])
    time.sleep(1)

    # l = 0.06
    # v = 0.05
    # a = 0.02
    # r = 0.01

    # print("Going to start")
    # # startj = [-1.0369790236102503, -1.0823825041400355, -2.211877171193258, 0.15503239631652832, 1.0424913167953491, -0.0012467543231409195]
    # # rob.movej(startj, acc=0.8, vel=0.4, wait=True)
    # initj = rob.getj() #result in radius
    # print("Initial joint configuration is ", initj)
    # t = rob.get_pose() #
    # print("Transformation from base to tcp is: ", t)

    p1 = [2.1390762329101562, -1.977527920399801, 2.101027011871338, -1.9881075064288538, -1.7569130102740687, 0.526220977306366]
    p2 = [2.0637872219085693, -1.7856486479388636, 2.19935941696167, -2.290964428578512, -1.7351067701922815, 0.45302730798721313]

    rob.movej(p1, acc=0.1, vel=0.2, wait=True)
    rob.movej(p2, acc=0.1, vel=0.2, wait=True)

    time.sleep(0.3)
    # wait()

    rospy.init_node('img_feedback_control', anonymous=True)
    mc = motion_controller()

    seam_tracking(rob)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        rob.stop()
        rob.close()

    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

