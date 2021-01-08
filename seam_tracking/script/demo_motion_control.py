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
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point
from cv_bridge import CvBridge, CvBridgeError 
import time
import urx
import logging
from scipy.spatial.transform import Rotation as R


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

        rospy.Subscriber("first_pose", Pose, self.callback_first_pose, queue_size=1)
        rospy.Subscriber("vel_vector", Point, self.callback_vel_vector, queue_size=1)

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

    def callback_first_pose(self, pose):

        global new_seal_start_pose

        print("received first pose")

        seal_start_pose = []

        pose_quat= [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        r = R.from_quat(pose_quat)
        pose_rotvec = r.as_rotvec()

        seal_start_pose.append(pose.position.x)
        seal_start_pose.append(pose.position.y)
        seal_start_pose.append(pose.position.z)
        seal_start_pose.append(pose_rotvec[0])
        seal_start_pose.append(pose_rotvec[1])
        seal_start_pose.append(pose_rotvec[2])

        new_seal_start_pose = uplift_z(seal_start_pose)
       

    def callback_vel_vector(self, vel_vec_point):

        global new_vel_vec
        print("received vel_vector")

        vel_vec = []
        vel_vec.append(vel_vec_point.x)
        vel_vec.append(vel_vec_point.y)
        vel_vec.append(vel_vec_point.z)

        new_vel_vec = normalise_vel(vel_vec)

        execute_trajectory()

def normalise_vel(vel_vec):

    sum = abs(vel_vec[0])+abs(vel_vec[1])+abs(vel_vec[2])
    # sum = math.sqrt(sum)
    new_vel_vec = np.array(vel_vec)*0.02/sum

    return new_vel_vec

def uplift_z(ur_pose):
    # print ur_poses[:,2]
    # print np.mean(ur_poses[:,2])
    r = R.from_rotvec(ur_pose[3:])
    Rot_matrix = r.as_dcm()
    new_z = Rot_matrix[:,2]
    new_y = Rot_matrix[:,1]
    #the length of pen 0.22m, cube dim: 0.02*0.02 unit:m
    # pen_length = 0.19 #planer
    offset_z = -0.01
    offset_y = 0.03
    # offset = 0.22 for setting camera as tcp
    displacement_z = offset_z*new_z
    displacement_y = offset_y*new_y
    new_ur_pose = ur_pose
    new_ur_pose[0] = ur_pose[0] + displacement_z[0] + displacement_y[0]
    new_ur_pose[1] = ur_pose[1] + displacement_z[1] + displacement_y[1]
    new_ur_pose[2] = ur_pose[2] + displacement_z[2] + displacement_y[2]
    new_ur_pose[3] = ur_pose[3]
    new_ur_pose[4] = ur_pose[4]
    new_ur_pose[5] = ur_pose[5]

    return new_ur_pose

def execute_trajectory():

    usr_input = raw_input("please enter to continue(q to quit): ")

    if not usr_input == 'q':
        seam_tracking()
    else:
        pass

def seam_tracking():
 
    global logi_pre, logi_post, rs_pre, rob, new_seal_start_pose, new_vel_vec

    rob = urx.Robot("192.168.0.2")
    tcp_torch = [-0.0002, -0.09302, 0.31, 0, 0, 0]
    rob.set_tcp(tcp_torch)
    time.sleep(0.2) #pause is essentail for tcp to take effect, min time is 0.1s

    rob.movel(new_seal_start_pose, acc=0.1, vel=0.1, wait=True)

    vx = new_vel_vec[0]
    vy = new_vel_vec[1]
    vz = new_vel_vec[2]

    vrx = 0
    vry = 0
    vrz = 0

    dx = 0
    dy = 0
    dz = 0

    a = 0.002
    t = 1

    # usr_input = raw_input("please enter to continue(q to quit): ")
    usr_input = ''

    if not usr_input == 'q':

        # rob.set_digital_out(0,True)
        rob.set_digital_out(0,False)
        time.sleep(1)

        start = time.time()

        while True:

            if logi_pre and rs_pre:

                rs_y_diff = rs_pre[-1]
                logi_y_diff = logi_pre[-1]
                y_scale = 0.001

                if not abs(rs_y_diff) <= 40 :
                    # print rs_y_diff
                    dy = rs_y_diff*y_scale
                    print "rs changing in y"
                    if abs(dy) > 0.002:
                        print "in if"
                        if dy>0:
                            dy=0.002
                        else:
                            dy=-0.002
                elif abs(rs_y_diff) < 40 and abs(logi_y_diff)>=10:
                    dy = -logi_y_diff*y_scale
                    print "logi changing in y"
                    # print dy
                    if abs(dy) > 0.002:
                        print "in if"
                        if dy>0:
                            dy=0.002
                        else:
                            dy=-0.002
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
                else:
                    dy = 0
                    print "all well"
                
            vel_speed = [vx+dx,vy+dy,vz+dz,vrx,vry,vrz]
            # print "moving with speed" + str(vel_speed)
            rob.speedl(vel_speed, a, t)

            end = time.time()
            print int(end-start) 

            if end-start > 25:
                rob.set_digital_out(0,False)
                raw_input("Press any to continue")
                rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)
                rob.stop()
                break

    if usr_input == 'q':
        rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)

# --------------- MAIN LOOP
def main(args):

    global logi_pre, logi_post, rs_pre, seal_start_pose, new_vel_vec
    logi_pre = []
    logi_post = []
    rs_pre = []
    seal_start_pose = []
    new_vel_vec =[]

    logging.basicConfig(level=logging.INFO)

    # wait()

    rospy.init_node('img_feedback_control', anonymous=True)
    mc = motion_controller()

    # seam_tracking(rob)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        rob.stop()
        rob.close()

    finally:
        rob.stop()
        rob.close()


    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)