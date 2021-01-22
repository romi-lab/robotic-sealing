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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridge, CvBridgeError 
# --- Define our Class
class image_converter:

    def __init__(self):

        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # publish processed img for visualization
        self.image_pub = rospy.Publisher("logi_info/logi_image", Image, queue_size=1)
        # publish pre roi information, includes: k, b, and d
        self.pre_roi_pub = rospy.Publisher("logi_info/pre_roi", Point, queue_size=1)
        self.post_roi_pub = rospy.Publisher("logi_info/post_roi", Point, queue_size=1)
        self.rviz_text_pub = rospy.Publisher("logi_info/rviz_text", Marker, queue_size=1)
        self.pre_roi_info = Point()
        self.post_roi_info = Point()
        self.rviz_text = Marker()
        self.bridge = CvBridge()

        # subscribe to get the image
        # usb camera image 1280*720
        rospy.Subscriber("usb_cam/image_raw", Image, self.callback_logi_image, queue_size=1, buff_size=52428800)
        rospy.Subscriber('start_process', Point, self.callback_start_process,  queue_size=1)


    def callback_start_process(self, Point):

        global reach_first_pose

        if Point.x == 1:

            reach_first_pose = True
        
        elif Point.x == -1:

            reach_first_pose = False


    def callback_logi_image(self, Image):

        global reach_first_pose

        try:
            opencv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)

        def show_text_in_rviz(error):

            text_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            if math.isnan(error)==False:
                text = 'Error: '+ str(round(error,2)) +' mm'
                if error > 1:
                    text_color =  ColorRGBA(1.0, 0, 0, 1.0)
            else:
                text = 'Error: not avaliable yet'

            self.rviz_text = Marker(
                            type=Marker.TEXT_VIEW_FACING,
                            id=0,
                            lifetime=rospy.Duration(1.5),
                            pose=Pose(Point(-0.4, -0.4, 0.1), Quaternion(0, 0, 0, 1)),
                            scale=Vector3(0.06, 0.06, 0.06),
                            header=Header(frame_id='base'),
                            color=text_color,
                            text=text)
            self.rviz_text_pub.publish(self.rviz_text)

        def image_processor(opencv_image):

            #get the basic info of image
            # rows >> width cols >> height channels >> e.g. RGB >> 3
            global frame

            frame = copy.deepcopy(opencv_image)
            (rows, cols, channels) = frame.shape
            height = rows # 720
            width = cols #1280

            limg = copy.deepcopy(frame)
            rimg = copy.deepcopy(frame)

            tip_width = int(width*75/160)
            tip_height = int(24/64*height) #240
            tip_pos = np.array([tip_width, tip_height])

            # cv2.line(frame, (tip_width, 0), (tip_width, int(height)), (0, 0, 0))
            cv2.circle(frame, (tip_width,tip_height), radius=5, color=(0, 255, 255), thickness=-1)
            
            # 2. pre zone
            error = process_pre_roi(limg, tip_pos)

            # 3. post zone
            # xr,yr,wr,hr = process_post_roi(rimg, tip_pos)

            # 4. create a black image for displaying information
            # info = display_info(d, xr, yr, hr, wr, tip_height)
            # images = np.hstack((frame, info))
            show_text_in_rviz(error)
            display_info_on_image(tip_height)
            images = frame

            if math.isnan(error)==False: # k,b,d
                
                self.pre_roi_info.x = 0
                self.pre_roi_info.y = 0
                self.pre_roi_info.z = error
                self.pre_roi_pub.publish(self.pre_roi_info)
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(images, "bgr8"))
            except CvBridgeError as e:
                print(e)
            
            cv2.waitKey(1)
        

        # if reach_first_pose:
        #     image_processor(opencv_image)
        # else:
        #     self.image_pub.publish(Image)
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
    
    global frame, k1_list, b1_list, count_nan, noedge, preroi_offset, k2_list, b2_list

    # 1. define left roi
    tip_width = tip_pos[0]
    tip_height = tip_pos[1]
    left_rectangle_center = np.array([tip_width, tip_height+100])
    # left_rectangle_height = 60
    left_rectangle_height = 60
    left_rectangle_width = 160
    cv2.rectangle(frame,(left_rectangle_center[0]- left_rectangle_width,left_rectangle_center[1]- left_rectangle_height),(left_rectangle_center[0]+ left_rectangle_width,left_rectangle_center[1]+ left_rectangle_height),(255,255,255),1)

    # 2. process left image
    # 2a. define left ROI
    limg = cv2.GaussianBlur(limg,(3,3),0)   
    # (xl1, yl1) denotes left up corner, (xl2,yl2) denotes right bottom corner
    # for the line function, the origin is left bottom corner
    # for image, the origin is left up corner
    yl1 = left_rectangle_center[1]- left_rectangle_height
    yl2 = left_rectangle_center[1]+ left_rectangle_height
    xl1 = left_rectangle_center[0]- left_rectangle_width
    xl2 = left_rectangle_center[0]+ left_rectangle_width
    left_rectangle_img = limg[yl1:yl2, xl1:xl2]

    # 2b. get the line function of the detected edge in left ROI 
   
    lk_list, lb_list = process_pre_roi_getkb(left_rectangle_img, tip_pos)
    error = np.nan

    # print lk_list, lb_list
    lk1 = lk_list[0]
    lb1 = lb_list[0]
    lk2 = lk_list[1]
    lb2 = lb_list[1]

    # for five consective frame
    # if all non then stop using the previous value


    if math.isnan(lk1)==False and math.isnan(lk2)==False:
        # first detetion
        noedge = False
        count_nan = 0
    
    if noedge == False:
        if math.isnan(lk1)==False and math.isnan(lk2)==False:
            count_nan = 0
        else:
            count_nan += 1
            lk1 = k1_list[-1]
            lb1 = b1_list[-1]
            lk2 = k2_list[-1]
            lb2 = b2_list[-1]
            if count_nan % 20 == 0:
                noedge = True
                print "no edge detected"

    if noedge == False:

        k1_list.append(lk1)
        b1_list.append(lb1)
        k2_list.append(lk2)
        b2_list.append(lb2)

        # 2c. filter the line
        lk1, lb1, lk2, lb2 = preroi_filter(lk1, lb1, lk2, lb2)

        y_offset = yl1
        x_offset = xl1
        lb2 = lb2+y_offset-lk2*x_offset
        b2 = yl1
        a2 = int((b2-lb2)/lk2)
        pa2 = a2
        pb2 = b2
        pb1 = yl2
        pa1 = int((pb1-lb2)/lk2)

        cv2.line(frame,(pa2,pb2),(pa1,pb1),(0,0,0),4)

        p2 = [a2,b2]

        lb1 = lb1+y_offset-lk1*x_offset
    
        cb1 = yl1
        ca1 = int((cb1-lb1)/lk1)

        # print "right edge detected"

        left_offset = 68

        a1 = a2 - left_offset
        b1 = b2
        p1 = [a1,b1]

        # cv2.line(frame,(a2,b2),(a1,b1),(0,0,0),4)
        
        diff = a1-ca1

        lb1 = lb1-lk1*diff

        cb1 = yl1
        cb2 = yl2
        ca1 = int((cb1-lb1)/lk1)

        ca2 = int((cb2-lb1)/lk1)

        ca1 = pa1 - left_offset
        ca2 = pa2 - left_offset
        cv2.line(frame,(ca1,cb1),(ca2,cb2),(0,255,0),2)
        cv2.circle(frame, (ca1, cb1), radius=20, color=(0, 255, 0), thickness=2)


        left_offset2 = 145
        a3 = a2-left_offset2
        b3 = b2
        p3 = [a3,b3]

        cv2.line(frame,(a2,b2),(a3,b3),(255,255,255),4)

        projection = ClosestPointOnLine(p2, p1, tip_pos)
        error = np.linalg.norm(projection-p1)

        check = np.linalg.norm(np.array(p3)-np.array(p2))

        #measured
        cv2.circle(frame, (int(projection[0]), int(projection[1])), radius=20, color=(0, 255, 255), thickness=2)
        cv2.line(frame,(tip_pos[0], tip_pos[1]), (int(projection[0]), int(projection[1])), color=(0, 255, 255), thickness=2)

        cv2.line(frame, (a1,b1), (int(projection[0]), int(projection[1])) , color=(255, 255, 0), thickness=1)

        # target
        # cv2.circle(frame, (a1,b1), radius=20, color=(0, 255, 0), thickness=2)
        
        scale = 0.129
        error = scale*error
        error_text = "error: " + str(round(error,3)) + " mm"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 1
        font_thickness = 2

        if error > 1:
            cv2.putText(frame, error_text,(720, 160), font, font_size,(0,0,255),font_thickness)
            cv2.line(frame, (int((a1+projection[0])/2), int((b1+projection[1])/2)), (750, 180) , color=(0,0,255), thickness=2)
        else:
            cv2.putText(frame, error_text,(720, 160), font, font_size,(255,255,0),font_thickness)
            cv2.line(frame, (int((a1+projection[0])/2), int((b1+projection[1])/2)), (750, 180) , color=(255, 255, 0), thickness=2)


    return error
        

def ClosestPointOnLine(a, b, p):
    a = np.array(a)
    b = np.array(b)
    p = np.array(p)
    ap = p-a
    ab = b-a
    result = a + np.dot(ap,ab)/np.dot(ab,ab) * ab
    return result

def process_pre_roi_getkb(limg, tip_pos):

    """
    Get the function of most likely edge using Hough Transfrmation

    Args:
        limg: original frame image
        tip_pos: point of action position

    Returns:
        k: deravative
        b: y-intercept
        (line equation: y = kx + b w.r.t left ROI)
    """
    limg_width = limg.shape[1]
    limg_height = limg.shape[0]
    new_p0 = np.array([int(limg_width/2), 0])

    gray = cv2.cvtColor(limg,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,15,20,apertureSize = 3) # apertureSize size of kernal
    lines = cv2.HoughLines(edges,1,np.pi/180,threshold=5)
    
    k1 = np.nan
    b1 = np.nan
    k2 = np.nan
    b2 = np.nan
    select_one = 1

    k_list = []
    b_list = []
    d_list = []
    d_with_sign_list = []
    x_pos_list = []
    new_k_list = []
    new_b_list = []

    if lines is not None:
        # filter out the line that is too far away from point of action
        slines = select_line(new_p0, lines)
        if slines is not None:
            for line in slines:
                for rho,theta in line:
                    n = np.array(slines).shape[0]
                    # only select the first one (strongest one) to be the edge
                    if select_one < min(6, n):
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
                        p1 = np.array([x1, y1])
                        p2 = np.array([x2, y2])
                        cv2.line(limg,(x1,y1),(x2,y2),(0,0,255),2)    
                        k, b = polar2cartesian(rho, theta, rotate90 = True) 
                        if not math.isnan(k):
                            k_list.append(k)
                            b_list.append(b)
                            d = np.cross(p2-p1,new_p0-p1)/np.linalg.norm(p2-p1)
                            d_with_sign_list.append(d)
                            d_list.append(abs(d))
                            # y = 0, x = -b/k
                            x_pos = -b/k
                            x_pos_list.append(x_pos)
                        # cv2.circle(limg, (int(x_pos), 0), radius=20, color=(0, 255, 255), thickness=2)
                    select_one = select_one +1 

            d_list = np.array(d_list)
            d_with_sign_list = np.array(d_with_sign_list)
            x_pos_list = np.array(x_pos_list)
            k_list = np.array(k_list)
            b_list = np.array(b_list)

            d_list = d_list[np.logical_not(np.isnan(d_list))]
            d_with_sign_list = d_with_sign_list[np.logical_not(np.isnan(d_with_sign_list))]
            x_pos_list = x_pos_list[np.logical_not(np.isnan(x_pos_list))]
            k_list = k_list[np.logical_not(np.isnan(k_list))]
            b_list = b_list[np.logical_not(np.isnan(b_list))]

            print k_list.shape[0], d_list.shape[0]

            inds1 = d_list.argsort()
            new_k_list = np.array(k_list)[inds1]
            new_b_list = np.array(b_list)[inds1]
            k1 = new_k_list[0]
            b1 = new_b_list[0]
            # inds2 = d_with_sign_list.argsort()
            # new_k_list = np.array(k_list)[inds2]
            # new_b_list = np.array(b_list)[inds2]
            # k2 = new_k_list[-1]
            # b2 = new_b_list[-1]

            inds2 = x_pos_list.argsort()
            new_k_list = np.array(k_list)[inds2]
            new_b_list = np.array(b_list)[inds2]
            k2 = new_k_list[-1]
            b2 = new_b_list[-1]

            print new_k_list, new_b_list

            x_pos = -b2/k2
            if not math.isnan(x_pos):
                cv2.circle(limg, (int(x_pos), 0), radius=20, color=(0, 255, 255), thickness=2)

            new_k_list = [k1, k2]
            new_b_list = [b1, b2]

    cv2.circle(limg, (new_p0[0],new_p0[1]), radius=3, color=(0, 255, 0), thickness=-1)
    cv2.imshow('roi', limg)
    cv2.imshow('edge', edges)

    return new_k_list, new_b_list

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
    angle_threshold = [np.pi/4, np.pi*3/4] 
    # the farest allowable distance from line to point of action
    distance_threshold = 100
    # the farest allowable distance from line y-intercept to point of action
    y_threshold = 20
    d_list = []

    for line in lines:
        for rho,theta in line:
            if not angle_threshold[0] <= theta <= angle_threshold[1]: 

                slines.append(line)

    return slines

def plotkb():
    """
    Plot k, b value
    """
    global k1_list, b1_list, f_k1_list, f_b1_list

    x = range(np.array(k1_list).shape[0])
    plt.ion()
    plt.show()
    plt.cla()
    
    plt.subplot(4,1,1)
    plt.plot(x, k1_list, color='r', linewidth=0.5)
    plt.title('slope: k_list')
    plt.subplot(4,1,2)
    plt.plot(x, f_k1_list, color='k', linewidth=0.5)
    plt.title('filtered_k_list')
    plt.subplot(4,1,3)
    plt.plot(x, b1_list,  color='r', linewidth=0.5)
    plt.title('y-intercept: b_list')
    plt.subplot(4,1,4)
    plt.plot(x, f_b1_list,  color='k', linewidth=0.5)
    plt.title('filtered_b_list')
        #     >>> plot(x, y, 'go--', linewidth=2, markersize=12)
        # >>> plot(x, y, color='green', marker='o', linestyle='dashed',
        # ...      linewidth=2, markersize=12)

    plt.tight_layout()
    plt.autoscale(enable=True)

    plt.draw()
    plt.pause(0.001)

def preroi_filter(k1, b1, k2, b2):
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
    global k1_list, b1_list, f_k1_list, f_b1_list, k2_list, b2_list, f_k2_list, f_b2_list

    # low pass filter
    new_k1 = k1
    new_b1 = b1
    new_k2 = k2
    new_b2 = b2
    # how close is this new measures to the previous one
    n = 20
    # how much the original data should be "corrected"q
    alpha = 0.1

    # k_weights=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2]
    k_weights = np.ones(n)
    
    if np.array(k1_list).shape[0]>n:
        new_k1 = np.average(np.array(k1_list)[-n-1:-1], weights=k_weights)
        new_b1 = np.average(np.array(b1_list)[-n-1:-1], weights=k_weights)
        # if abs(new_k1-k1)>10 or abs(k1-k1_list[-2])>10 or abs(new_k1-k1_list[-n])>10:
        #     # print "line change dtected:  " + str(abs(new_k-lk))
        #     new_k1 = k1
        #     new_b1 = b1
    
    f_k1_list.append(new_k1)
    f_b1_list.append(new_b1)
    k1_list[-1] = alpha*new_k1 + (1-alpha)*k1_list[-1]
    b1_list[-1] = alpha*new_b1 + (1-alpha)*b1_list[-1]
    
    if np.array(k2_list).shape[0]>n:
        new_k2 = np.average(np.array(k2_list)[-n-1:-1], weights=k_weights)
        new_b2 = np.average(np.array(b2_list)[-n-1:-1], weights=k_weights)
        # if abs(new_k2-k2)>10 or abs(k2-k2_list[-2])>10 or abs(new_k2-k2_list[-n])>10:
        #     # print "line change dtected:  " + str(abs(new_k-lk))
        #     new_k2 = k2
        #     new_b2 = b2

    f_k2_list.append(new_k2)
    f_b2_list.append(new_b2)
    k2_list[-1] = alpha*new_k2 + (1-alpha)*k2_list[-1]
    b2_list[-1] = alpha*new_b2 + (1-alpha)*b2_list[-1]

    # plotkb()

    return new_k1, new_b1, new_k2, new_b2

def display_info_on_image(tip_height):
    """
    Dispaly useful information
    """

    global frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    i_in = 580
    b_in = 30
    s_in = 30
    left = 50
    left_offset=20
    upper_offset = 7
    font_size = 1
    font_thickness = 2

    cv2.circle(frame, (left-left_offset,i_in+b_in-upper_offset), radius=14, color=(0, 255, 0), thickness=2)
    cv2.putText(frame,' target position',(left,i_in+b_in), font, font_size,(0,255,0),font_thickness)

    cv2.circle(frame, (left-left_offset,i_in+2*b_in+s_in-upper_offset), radius=14, color=(0, 255, 255), thickness=2)
    cv2.putText(frame,' measured position',(left,i_in+2*b_in+s_in), font, font_size,(0,255,255),font_thickness)

# --------------- MAIN LOOP
def main(args):

    rospy.init_node('logi_img_processing', anonymous=True)
    global k1_list, b1_list, f_k1_list, f_b1_list, count_nan, noedge, preroi_offset, postroi_offset, reach_first_pose, k2_list, b2_list, f_k2_list, f_b2_list

    # window offset
    k1_list = []
    b1_list = []
    f_k1_list = []
    f_b1_list = []
    k2_list = []
    b2_list = []
    f_k2_list = []
    f_b2_list = []
    count_nan = 0
    noedge = True
    preroi_offset = 150
    postroi_offset = -110
    reach_first_pose = False

    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

