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
from kf_book.mkf_internal import plot_track
import kf_book.book_plots as book_plots
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
        
            global logi_pre, logi_post, rs_pre, rob, new_seal_start_pose, new_vel_vec, new_seal_end_pose

            rob = urx.Robot("192.168.0.2")
            tcp_torch = [-0.0002, -0.09216, 0.32202, 0, 0, 0]
            rob.set_tcp(tcp_torch)
            time.sleep(0.1) #pause is essentail for tcp to take effect, min time is 0.1s

            rob.movel(new_seal_start_pose, acc=0.1, vel=0.1, wait=True)

            self.start = Point()
            self.start.x = 1
            self.start.y = 0
            self.start.z = 0
            self.pub_start_process.publish(self.start)

            vel_scale = 0.015

            # new_vel_vec = vel_scale* new_vel_vec

            translation_speed = vel_scale*0.5

            pose_diff = np.linalg.norm(np.array(new_seal_end_pose[:3])-np.array(new_seal_start_pose[:3]))
            total_time = pose_diff / translation_speed
            
            print "total time is (s)"
            print total_time

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

                        print "working"

                        logi_y_diff = logi_pre[-1]
                        angle_diff = logi_pre[0]
                        y_scale = vel_scale*0.5
                        a_scale = vel_scale*0.8

                        if abs(logi_y_diff) > 2:
                            # print rs_y_diff
                            dx = (logi_y_diff)*y_scale
                            # if logi_y_diff > 0:
                            #     dy = -(logi_y_diff-0.5)*y_scale
                            # else:
                            #     dy = -(logi_y_diff+0.5)*y_scale
                        elif abs(logi_y_diff) > 1 :
                            dx = (logi_y_diff)*y_scale*0.4
                        elif abs(logi_y_diff) > 0.5 :
                            dx = (logi_y_diff)*y_scale*0.2
                        elif abs(logi_y_diff) > 0.2 :
                            dx = (logi_y_diff)*y_scale*0.1
                        else:
                            dx = 0
                            print "all well"
                        
                        if abs(dx) > vel_scale*0.5:
                            if dx>0:
                                dx=vel_scale*0.5
                            else:
                                dx=-vel_scale*0.5

                        if abs(angle_diff) > 1:

                            drz = a_scale*angle_diff
                        
                        if abs(drz) > a_scale*20:
                            if drz>0:
                                drz= a_scale*20
                            else:
                                drz= -a_scale*20
                        
                        print "moving at z = "+str(drz)
                        
                    vel_speed = [vx+dx,vy+dy,vz+dz,vrx+drx,vry+dry,vrz+drz]
                    rob.speedl_tool(vel_speed, a, t)

                    end = time.time()
                    print int(end-start) 

                    if end-start > total_time:
                        self.start = Point()
                        self.start.x = -1
                        self.start.y = 0
                        self.start.z = 0
                        self.pub_start_process.publish(self.start)
                        rob.set_digital_out(0,False)
                        # raw_input("Press any to continue")
                        rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)
                        rob.stop()
                        break

            if usr_input == 'q':
                rob.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)

        seam_tracking(self)

        # print("received measured error")
    
    def callback_logi_info_pre_roi(self, Point):

        global logi_pre

        # print("received measured error")
        
        logi_pre_k = Point.x
        logi_pre_b = Point.y
        logi_pre_d = Point.z

        logi_pre = [logi_pre_k, logi_pre_b, logi_pre_d]

    def callback_logi_image(self, Image):

        try:
            opencv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)

        def image_processor(opencv_image):

            #get the basic info of image
            # rows >> width cols >> height channels >> e.g. RGB >> 3
            global frame

            frame = copy.deepcopy(opencv_image)
            (rows, cols, channels) = frame.shape
            height = rows # 720
            width = cols #1280

            limg = copy.deepcopy(frame)

            tip_width = int(width*75/160)
            tip_height = int(24/64*height) #240
            tip_pos = np.array([tip_width, tip_height])
            
            images = frame
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(images, "bgr8"))
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

dt = .1
x = np.array([0., 0.]) 
kf = pos_vel_filter(x, P=500, R=5, Q=0.1, dt=dt)

def run(x0=(0.,0.), P=500, R=0, Q=0, dt=1.0, 
        track=None, zs=None,
        count=0, do_plot=True, **kwargs):
    """
    track is the actual position of the dog, zs are the 
    corresponding measurements. 
    """

    # Simulate dog if no data provided. 
    if zs is None:
        track, zs = compute_dog_data(R, Q, count)

    # create the Kalman filter
    kf = pos_vel_filter(x0, R=R, P=P, Q=Q, dt=dt)  

    # run the kalman filter and store the results
    xs, cov = [], []
    for z in zs:
        kf.predict()
        kf.update(z)
        xs.append(kf.x)
        cov.append(kf.P)

    xs, cov = np.array(xs), np.array(cov)
    if do_plot:
        plot_track(xs[:, 0], track, zs, cov, **kwargs)
    return xs, cov

P = np.diag([500., 49.])
Ms, Ps = run(count=50, R=10, Q=0.01, P=P)

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

    global logi_pre
    logi_pre = []
    z = []

    logging.basicConfig(level=logging.INFO)

    # wait()

    rospy.init_node('kf_feedback', anonymous=True)
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