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
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError 

# --- Define our Class
class image_converter:

    def __init__(self):

        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # publish processed img for visualization
        self.image_pub = rospy.Publisher("rs_info/rs_rgb_image", Image, queue_size=1)
        # publish pre roi information, includes: k, b, and d
        self.pre_roi_pub = rospy.Publisher("rs_info/rgb/pre_roi", Point, queue_size=1)
        self.pre_roi_info = Point()
        self.bridge = CvBridge()

        # subscribe to get the image
        # usb camera image 640*480
        rospy.Subscriber("camera/color/image_raw", Image, self.callback_rs_rgb_image, queue_size=1, buff_size=52428800)

    def callback_rs_rgb_image(self, Image):

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
            height = rows # 480
            width = cols #640

            limg = copy.deepcopy(frame)

            tip_width = int(width*107/160)
            tip_height = int(58/64*height) #240
            tip_pos = np.array([tip_width, tip_height])

            tip_end_width = int(width*109/160)
            tip_end_height = int(height)
            tip_end_pos = np.array([tip_end_width, tip_end_height])

            cv2.line(frame, (tip_width, 0), (tip_width, int(height)), (0, 0, 0))
            cv2.circle(frame, (tip_width,tip_height), radius=3, color=(0, 255, 0), thickness=-1)
            # cv2.circle(frame, (tip_end_width,tip_end_height), radius=5, color=(0, 255, 0), thickness=-1)
            cv2.line(frame, (tip_width, tip_height), (tip_end_width, tip_end_height), (0, 0, 0), thickness=2)
        
            # 2. pre zone
            k,b,d = process_pre_roi(limg, tip_pos)

            # 4. create a black image for displaying information
            # cv2.imshow('frame', frame)
            # info = display_info(d, tip_height)
            # images = np.hstack((frame, info))

            display_info_on_image(d, tip_height)
            images = frame

            if math.isnan(k)==False and math.isnan(d)==False: # k,b,d
                self.pre_roi_info.x = k
                self.pre_roi_info.y = b
                self.pre_roi_info.z = d
                self.pre_roi_pub.publish(self.pre_roi_info)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(images, "bgr8"))
            except CvBridgeError as e:
                print(e)

        image_processor(opencv_image)


def process_pre_roi(limg, tip_pos):

    """
    Define pre sealing roi and methods to process the image

    Args:
        limg: original frame image
        tip_pos: point of action position

    Returns:
        k: deravative
        b: y-intercept
        (line equation: y = kx + b w.r.t original frame)
    """
    
    global frame, k_list, b_list, count_nan, noedge, preroi_offset

    # 1. define left roi
    tip_width = tip_pos[0]
    tip_height = tip_pos[1]
    pre_roi_rectangle_center = np.array([tip_width, tip_height+preroi_offset])
    pre_roi_rectangle_width = 80
    pre_roi_rectangle_height = 80
    cv2.rectangle(frame,(pre_roi_rectangle_center[0]- pre_roi_rectangle_width,pre_roi_rectangle_center[1]- pre_roi_rectangle_height),(pre_roi_rectangle_center[0]+ pre_roi_rectangle_width,pre_roi_rectangle_center[1]+ pre_roi_rectangle_height),(255,255,255),1)

    # 2. process left image
    # 2a. define left ROI
    limg = cv2.GaussianBlur(limg,(3,3),0)   
    # (xl1, yl1) denotes left up corner, (xl2,yl2) denotes right bottom corner
    # for the line function, the origin is left bottom corner
    # for image, the origin is left up corner
    yl1 = pre_roi_rectangle_center[1]- pre_roi_rectangle_height
    yl2 = pre_roi_rectangle_center[1]+ pre_roi_rectangle_height
    xl1 = pre_roi_rectangle_center[0]- pre_roi_rectangle_width
    xl2 = pre_roi_rectangle_center[0]+ pre_roi_rectangle_width
    left_rectangle_img = limg[yl1:yl2, xl1:xl2]

    # 2b. get the line function of the detected edge in left ROI 
    d = np.nan
    limg_width = left_rectangle_img.shape[1]
    limg_height = left_rectangle_img.shape[0]
    new_p0 = np.array([int(limg_width/2), int(limg_height)-(pre_roi_rectangle_height+preroi_offset)])
    lk, lb = process_pre_roi_getkb(left_rectangle_img, tip_pos, new_p0)

    # for five consective frame
    # if all non then stop using the previous value
    if noedge == True and math.isnan(lk)==False:
        # first detetion
        noedge = False
        count_nan = 0
    
    elif noedge == False:
        if not math.isnan(lk):
            count_nan = 0
        else:
            count_nan += 1
            lk = k_list[-1]
            lb = b_list[-1]
            if count_nan % 20 == 0:
                noedge = True

    if noedge == False and math.isnan(lk)==False and math.isnan(lb)==False:

        k_list.append(lk)
        b_list.append(lb)

        # 2c. filter the line
        lk, lb = preroi_filter(lk, lb)

        print lk, lb

        # 2d. lb = lb+y_offset-lk*x_offset remap the function from ROI to original frame
        y_offset = yl1
        x_offset = xl1
        lb = lb+y_offset-lk*x_offset
        b1 = yl1
        b2 = yl2
        a1 = int((b1-lb)/lk)
        a2 = int((b2-lb)/lk)
        cv2.line(frame,(a1,b1),(a2,b2),(0,0,255),2)

        # 2e. calculate the error (distance between the line to the point of action (tip))
        p1 = np.array([a1, b1])
        p2 = np.array([a2, b2])
        d = np.cross(p2-p1,tip_pos-p1)/np.linalg.norm(p2-p1)
        
    
    return lk, lb, d

def process_pre_roi_getkb(limg, tip_pos, new_p0):

    """
    Get the function of most likely edge using Hough Transfrmation

    Args:
        limg: original frame image
        tip_pos: point of action position
        new_p0: position of tip w.r.t the new window

    Returns:
        k: deravative
        b: y-intercept
        (line equation: y = kx + b w.r.t left ROI)
    """

    # cv2.circle(limg, (new_p0[0],new_p0[1]), radius=3, color=(0, 255, 0), thickness=-1)

    gray = cv2.cvtColor(limg,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,70,100,apertureSize = 3) #apertureSize size of kernal
    lines = cv2.HoughLines(edges,1,np.pi/180,threshold=30)

    k = np.nan
    b = np.nan
    select_one = 1

    if lines is not None:
        # filter out the line that is too far away from point of action
        slines = select_line(new_p0, lines)
        if slines is not None:
            for line in slines:
                for rho,theta in line:
                    # only select the first one (strongest one) to be the edge
                    if select_one == 1:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
                        cv2.line(limg,(x1,y1),(x2,y2),(0,0,255),2)    
                        k, b = polar2cartesian(rho, theta, rotate90 = True) 
                        select_one = select_one +1 

    # cv2.imshow('roi', limg)
    # cv2.imshow('edge', edges)

    return k, b

def polar2cartesian(rho, theta_rad, rotate90 = True):
    """
    Converts line equation from polar to cartesian coordinates

    Args:
        rho: input line rho
        theta_rad: input line theta
        rotate90: output line perpendicular to the input line

    Returns:
        m: slope of the line
        For horizontal line: m = 0
        For vertical line: m = np.nan
        b: intercept when x=0
    """
    x = np.cos(theta_rad) * rho
    y = np.sin(theta_rad) * rho
    m = np.nan
    if not np.isclose(x, 0.0):
        m = y / x
    if rotate90:
        if m is np.nan:
            m = 0.0
        elif np.isclose(m, 0.0):
            m = np.nan
        else:
            m = -1.0 / m
    b = 0.0
    if m is not np.nan:
        b = y - m * x

    return m, b

def select_line(p0, lines):

    """
    Filter the lines 

    Args:
        p0: position of point of action
        lines: all the lines(edges) detected

    Returns:
        slines: lines that meet the tolerance (order: from strong to weak according to Hough Transfrmation)
    """

    slines = []
    # angle of line, 0 for vertical, pi/2 for horizonal
    angle_threshold = [np.pi/3, np.pi*2/3] 
    # the farest allowable distance from line to point of action
    distance_threshold = 80
    # the farest allowable distance from line y-intercept to point of action
    y_threshold = 40
    # d_list = []

    for line in lines:
        for rho,theta in line:
            if not angle_threshold[0] <= theta <= angle_threshold[1]: 
                if not math.isnan(theta):
                    slines.append(line)
                
    return slines

def plotkb():
    """
    Plot k, b value
    """
    global k_list, b_list, f_k_list, f_b_list

    x = range(np.array(k_list).shape[0])
    plt.ion()
    plt.show()
    plt.cla()
    
    plt.subplot(4,1,1)
    plt.plot(x, k_list, color='r', linewidth=0.5)
    plt.title('slope: k_list')
    plt.subplot(4,1,2)
    plt.plot(x, f_k_list, color='k', linewidth=0.5)
    plt.title('filtered_k_list')
    plt.subplot(4,1,3)
    plt.plot(x, b_list,  color='r', linewidth=0.5)
    plt.title('y-intercept: b_list')
    plt.subplot(4,1,4)
    plt.plot(x, f_b_list,  color='k', linewidth=0.5)
    plt.title('filtered_b_list')
        #     >>> plot(x, y, 'go--', linewidth=2, markersize=12)
        # >>> plot(x, y, color='green', marker='o', linestyle='dashed',
        # ...      linewidth=2, markersize=12)

    plt.tight_layout()
    plt.autoscale(enable=True)

    plt.draw()
    plt.pause(0.001)

def preroi_filter(lk, lb):
    """
    Filter the data using first order low-pass filter, two parameters to adjust the result
    alpha: how much the original data should be "corrected", 0 for no correction
    n: how close should this new measures be to the previous one, for averaging the value

    Args:
        lk: slope
        lb: y-intercept

    Returns:
        new_k: new slope
        new_b: new y-intercept
    """
    global k_list, b_list, f_k_list, f_b_list

    # low pass filter
    new_k = lk
    new_b = lb
    # how close is this new measures to the previous one
    n = 20

    # k_weights=[1,1,1,1,1,2,2,2,2,2]
    k_weights = np.ones(n)

    if np.array(k_list).shape[0]>n:
        new_k = np.average(np.array(k_list)[-n-1:-1], weights=k_weights)
        new_b = np.average(np.array(b_list)[-n-1:-1], weights=k_weights)
        if abs(new_k-lk)>10 or abs(lk-k_list[-2])>8 or abs(new_k-k_list[-n])>15 or new_k==0:
            # print "line change dtected:  " + str(abs(new_k-lk))
            new_k = lk
            new_b = lb
            
    f_k_list.append(new_k)
    f_b_list.append(new_b)

    # how much the original data shoud be corrected
    alpha = 0
    k_list[-1] = alpha*new_k + (1-alpha)*k_list[-1]
    b_list[-1] = alpha*new_b + (1-alpha)*b_list[-1]

    # plotkb()

    return new_k, new_b 

def display_info(d, tip_height):
    """
    Dispaly useful information
    """
    note = np.zeros((480,400,3), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    i_in = 20
    b_in = 50
    s_in = 30
    cv2.putText(note,'Feedback',(10,i_in), font, 0.5,(255,255,255),1)

    cv2.putText(note,'Green: point of action (target)',(10,i_in+b_in), font, 0.5,(0,255,0),1)

    cv2.putText(note,'Box',(10,i_in+2*b_in), font, 0.5,(255,255,255),1)
    cv2.putText(note,'Red line: edge (detected)',(10,i_in+2*b_in+s_in), font, 0.5,(0,0,255),1)
    
    if not math.isnan(d):
        cv2.putText(note,'Distance to target: '+ str(round(d,2)) +' pixel (error)',(10,i_in+2*b_in+2*s_in), font, 0.5,(0,0,255),1)

    return note

def display_info_on_image(d, tip_height):
    """
    Dispaly useful information
    """

    global frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    i_in = 300
    b_in = 30
    s_in = 30
    font_size = 0.5
    font_thickness = 1

    cv2.putText(frame,'Green: point of action (target)',(10,i_in+b_in), font, font_size,(0,255,0),font_thickness)
    cv2.putText(frame,'Red line: edge (detected)',(10,i_in+2*b_in+s_in), font, font_size,(0,0,255),font_thickness)
    if not math.isnan(d):
        cv2.putText(frame,'Distance to target: '+ str(round(d,2)) +' pixel (error)',(10,i_in+2*b_in+2*s_in), font, font_size,(0,0,255),font_thickness)


# --------------- MAIN LOOP
def main(args):

    rospy.init_node('rs_rgb_processing', anonymous=True)

    global k_list, b_list, f_k_list, f_b_list, count_nan, noedge, preroi_offset, postroi_offset
    # window offset
    k_list = []
    b_list = []
    f_k_list = []
    f_b_list = []
    count_nan = 0
    noedge = True
    preroi_offset = -100

    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

