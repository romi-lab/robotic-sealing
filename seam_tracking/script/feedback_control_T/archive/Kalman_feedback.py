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
from filterpy.kalman import predict, update
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from numpy.random import randn


def wait():
    raw_input("please enter to continue: ")

# --- Define our Class
class motion_controller:

    def __init__(self):
   
        self.pub_start_process = rospy.Publisher('start_process', Point, queue_size=1)
        self.image_pub = rospy.Publisher("kf_image", Image, queue_size=1)
        self.bridge = CvBridge()

        # subscribe to get the info for feedback control
        # pre roi information, includes: theta(in degree) -  slope of line, 0, and error(d)
        rospy.Subscriber("logi_info/pre_roi", Point, self.callback_logi_info_pre_roi, queue_size=1, buff_size=52428800)
        rospy.Subscriber("logi_info/logi_image", Image, self.callback_logi_image, queue_size=1, buff_size=52428800)
        rospy.Subscriber("execute_seam_tracking", Point, self.callback_execute_seam_tracking, queue_size=1)
    
    def callback_execute_seam_tracking(self, execution_point):

        rospy.loginfo("executing")

        def execute_trajectory(self):

            usr_input = raw_input("please enter to continue(q to quit): ")

            if not usr_input == 'q':
                seam_tracking(self)
            else:
                pass

        def seam_tracking(self):
        
            global kf, logi_pre

            p1 = [-1.2291749159442347, -1.6868732611285608, -1.5637076536761683, -1.4049976507769983, 1.506557583808899, -1.1270230452166956]
            p2 = [0.1475816316024325, -0.5758160444905072, 0.01675402360559264, 2.043148886497484, -2.2799701718480754, 0.03938403385889866]

            rob = urx.Robot("192.168.0.2")
            tcp_torch = [-0.0002, -0.09216, 0.32202, 0, 0, 0]
            rob.set_tcp(tcp_torch)
            time.sleep(0.1) #pause is essentail for tcp to take effect, min time is 0.1s

            rob.movej(p1, acc=0.1, vel=0.2, wait=True)
            rob.movel(p2, acc=0.1, vel=0.1, wait=True)

            self.start = Point()
            self.start.x = 1
            self.start.y = 0
            self.start.z = 0
            self.pub_start_process.publish(self.start)

            vel_scale = 0.005 # 0.015

            # new_vel_vec = vel_scale* new_vel_vec

            translation_speed = vel_scale*0.5

            vx = 0
            vy = translation_speed
            vz = 0

            vrx = 0
            vry = 0
            vrz = 0

            dx = 0
            dy = 0
            dz = 0

            drx = 0
            dry = 0
            drz = 0

            a = vel_scale*5
            t = 0.1

            # usr_input = raw_input("please enter to continue(q to quit): ")
            print("start feedback control")
            usr_input = ''

            if not usr_input == 'q':

                # rob.set_digital_out(0,True)
                rob.set_digital_out(0,False)
                time.sleep(1)

                start = time.time()

                while True:

                    if logi_pre:

                        # print "working"

                        logi_y_diff = logi_pre[-1]
                        angle_diff = logi_pre[0]
                        y_scale = vel_scale*0.5
                        a_scale = vel_scale*0.8

                        # if abs(logi_y_diff) > 2:
                        #     # print rs_y_diff
                        #     dx = (logi_y_diff)*y_scale
                        #     # if logi_y_diff > 0:
                        #     #     dy = -(logi_y_diff-0.5)*y_scale
                        #     # else:
                        #     #     dy = -(logi_y_diff+0.5)*y_scale
                        # elif abs(logi_y_diff) > 1 :
                        #     dx = (logi_y_diff)*y_scale*0.4
                        # elif abs(logi_y_diff) > 0.5 :
                        #     dx = (logi_y_diff)*y_scale*0.2
                        # elif abs(logi_y_diff) > 0.2 :
                        #     dx = (logi_y_diff)*y_scale*0.1
                        # else:
                        #     dx = 0
                        #     print "all well"
                        
                        # if abs(dx) > vel_scale*0.5:
                        #     if dx>0:
                        #         dx=vel_scale*0.5
                        #     else:
                        #         dx=-vel_scale*0.5

                        if abs(angle_diff) > 1:

                            drz = a_scale*angle_diff
                        
                        if abs(drz) > a_scale*20:
                            if drz>0:
                                drz= a_scale*20
                            else:
                                drz= -a_scale*20
                        

                    vel_speed = [vx+dx,vy+dy,vz+dz,vrx+drx,vry+dry,vrz+drz]
                    rob.speedl_tool(vel_speed, a, t)
                    # kf.x[1] = -(vx+dx)*500

                    # print('\nvel changed')
                    # print kf.x[1]

                    end = time.time()
                    # print int(end-start) 

                    if end-start > 60:
                        self.start = Point()
                        self.start.x = -1
                        self.start.y = 0
                        self.start.z = 0
                        self.pub_start_process.publish(self.start)
                        rob.set_digital_out(0,False)
                        # raw_input("Press any to continue")
                        rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)
                        rob.stop()
                        rob.close()
                        break

            if usr_input == 'q':
                rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)

        seam_tracking(self)

        # print("received measured error")
    
    def callback_logi_info_pre_roi(self, Point):

        global logi_pre, z_list, xs_list, cov_list, kf, error_d, predict_x
        
        logi_pre_angle = Point.x
        logi_pre_reserve = Point.y
        logi_pre_d = Point.z

        logi_pre = [logi_pre_angle, logi_pre_reserve, logi_pre_d]
        
        # run the kalman filter and store the results

        # P = np.diag([100., 400])
        # R = 1000

        measured_value = logi_pre[-1]
        print("=========KF=======")
        print('current position (p): ' + str(kf.x[0]))
        print('current velocity (v): ' + str(kf.x[1]))
        print('duration: 0.1s' )
        
        kf.predict()
        print('predicted position p = p+vt: '+str(kf.x[0]))
        predict_x.append(kf.x)
        print('measured value: ' + str(measured_value))
        kf.update(logi_pre[-1])
        print('estimated position (p): ' + str(kf.x[0]))
        xs_list.append(kf.x)
        cov_list.append(kf.P)
        z_list.append(logi_pre[-1])

        # plot it
        scale = 0.129
        error_d = kf.x[0]/scale


        do_plot = True
        if do_plot:

            x = range(np.array(z_list).shape[0])
            plt.ion()
            plt.show()
            plt.cla()
            
            plt.plot(x, np.array(xs_list)[:, 0], color='r', linewidth=0.5, label='kf')
            plt.plot(x, np.array(z_list), color='k', linewidth=0.5, label='measure')

            plt.tight_layout()
            plt.autoscale(enable=True)
            plt.legend()

            plt.draw()
            plt.pause(0.001)

    def callback_logi_image(self, Image):

        try:
            opencv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)

        def image_processor(opencv_image):

            #get the basic info of image
            # rows >> width cols >> height channels >> e.g. RGB >> 3
            global frame, error_d

            frame = copy.deepcopy(opencv_image)
            (rows, cols, channels) = frame.shape
            height = rows # 720
            width = cols #1280

            tip_width = int(width*75/160)
            tip_height = int(19/64*height) #240
            tip_pos = np.array([tip_width, tip_height])

            new_x = int(error_d + tip_width)

            left_rectangle_center = np.array([tip_width, tip_height+100])
            # left_rectangle_height = 60
            left_rectangle_height = 60
            left_rectangle_width = 200
            cv2.rectangle(frame,(left_rectangle_center[0]- left_rectangle_width,left_rectangle_center[1]- left_rectangle_height),(left_rectangle_center[0]+ left_rectangle_width,left_rectangle_center[1]+ left_rectangle_height),(255,255,255),1)

            yl1 = left_rectangle_center[1]- left_rectangle_height
            yl2 = left_rectangle_center[1]+ left_rectangle_height
            xl1 = left_rectangle_center[0]- left_rectangle_width
            xl2 = left_rectangle_center[0]+ left_rectangle_width

            cv2.circle(frame, (new_x, yl1), radius=10, color=(0, 255, 255), thickness=-1)

            images = frame
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(images, "bgr8"))
                rospy.sleep(0.1)
            except CvBridgeError as e:
                print(e)
            
            cv2.waitKey(1)
        
        image_processor(opencv_image)


def pos_vel_filter(x, P, R, Q=0., dt=1.0):
    """ Returns a KalmanFilter which implements a
    constant velocity model for a state [x dx].T
    """
    
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.array([x[0], x[1]]) # location and velocity
    kf.F = np.array([[1., dt],
                     [0.,  1.]])  # state transition matrix
    kf.H = np.array([[1., 0]])    # Measurement function
    kf.R *= R                     # measurement uncertainty
    if np.isscalar(P):
        kf.P *= P                 # covariance matrix 
    else:
        kf.P[:] = P               # [:] makes deep copy
    if np.isscalar(Q):
        kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q)
    else:
        kf.Q[:] = Q
    return kf

def normalise_vel(vel_vec):

    sum = abs(vel_vec[0])+abs(vel_vec[1])+abs(vel_vec[2])
    # sum = math.sqrt(sum)
    new_vel_vec = np.array(vel_vec)/sum

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
    offset_z = -0.005
    offset_y = 0.0
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


# --------------- MAIN LOOP
def main(args):

    global logi_pre, z_list, xs_list, cov_list, kf, predict_x
    logi_pre = []
    z_list = []
    xs_list, cov_list, predict_x = [], [], []

    dt = .1
    x0 = np.array([0., 0.])  # p = 0, v=0
    P = np.diag([400., 400])
    R = 100
    kf = pos_vel_filter(x0, P=P, R=R, Q=400, dt=dt)

    logging.basicConfig(level=logging.INFO)

    # wait()

    rospy.init_node('kf_feedback', anonymous=True)
    mc = motion_controller()

    # seam_tracking(rob)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)