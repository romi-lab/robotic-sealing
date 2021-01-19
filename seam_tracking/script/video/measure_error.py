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
                text = 'Error: '+ str(round(error,2)) +'mm'
                if error > 1:
                    text_color =  ColorRGBA(1.0, 0, 0, 1.0)
                if error > 0.5:
                    text_color =  ColorRGBA(1.0, 1.0, 0, 1.0)
            else:
                text = 'Error: not avaliable yet'

            self.rviz_text = Marker(
                            type=Marker.TEXT_VIEW_FACING,
                            id=0,
                            lifetime=rospy.Duration(1.5),
                            pose=Pose(Point(-0.02, -0.9, 0.05), Quaternion(0, 0, 0, 1)),
                            scale=Vector3(0.04, 0.04, 0.04),
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

            tip_width = int(width*75/160)
            tip_height = int(24/64*height) #240
            tip_pos = np.array([tip_width, tip_height])

            # cv2.line(frame, (tip_width, 0), (tip_width, int(height)), (0, 0, 0))
            cv2.circle(frame, (tip_width,tip_height), radius=5, color=(0, 255, 255), thickness=-1)
            
            # 2. pre zone
            error, seam_angle = process_pre_roi(limg, tip_pos)

            error_angle = seam_angle - 90

            # 4. create a black image for displaying information
            show_text_in_rviz(abs(error))
            display_info_on_image(tip_height)
            images = frame

            if math.isnan(error)==False: # k,b,d
                
                self.pre_roi_info.x = error_angle
                self.pre_roi_info.y = 0
                self.pre_roi_info.z = error
                self.pre_roi_pub.publish(self.pre_roi_info)
                # print("error in angle is "+str(error_angle))
            
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
    
    global frame, k_list, b_list, count_nan, noedge, preroi_offset, point_list

    # 1. define left roi
    tip_width = tip_pos[0]
    tip_height = tip_pos[1]
    left_rectangle_center = np.array([tip_width, tip_height+100])
    # left_rectangle_height = 60
    left_rectangle_height = 60
    left_rectangle_width = 200
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
   
    lk, lb= process_pre_roi_getkb(left_rectangle_img, tip_pos)
    error = np.nan

    # print lk_list, lb_list

    # for five consective frame
    # if all non then stop using the previous value

    if math.isnan(lk)==False:
        # first detetion
        noedge = False
        count_nan = 0
    
    if noedge == False:
        if math.isnan(lk)==False:
            count_nan = 0
        else:
            count_nan += 1
            lk = k_list[-1]
            lb = b_list[-1]
            print("count nan")
            if count_nan % 20 == 0:
                noedge = True
                print "no edge detected"

    if noedge == False:

        k_list.append(lk)
        b_list.append(lb)

        if math.isnan(lk):
            print("lk is nan")

        # 2c. filter the line
        # lk, lb = preroi_filter(lk, lb)

        y_offset = yl1
        x_offset = xl1
        lb = lb+y_offset-lk*x_offset
        right_point_up_y = yl1
        right_point_up_x = int((right_point_up_y-lb)/lk)
        right_point_dn_y = yl2
        right_point_dn_x = int((right_point_dn_y-lb)/lk)

        right_point = [right_point_up_x, right_point_dn_x]

        point_list.append(right_point)

        right_point_up_x, right_point_dn_x = preroi_point_filter()

        # cv2.line(frame,(right_point_up_x,right_point_up_y), (right_point_dn_x,right_point_dn_y),(0,0,0),4)

        p1 = [right_point_up_x,right_point_up_y]

        # cv2.line(frame,(a2,b2),(a1,b1),(0,0,0),4)

        left_offset = 70
        
        center_point_up_y = yl1
        center_point_dn_y = yl2
        center_point_up_x = right_point_up_x - left_offset
        center_point_dn_x = right_point_dn_x - left_offset
        cv2.line(frame,(center_point_up_x,center_point_up_y),(center_point_dn_x,center_point_dn_y),(0,255,0),2)
        cv2.circle(frame, (center_point_up_x, center_point_up_y), radius=20, color=(0, 255, 0), thickness=2)

        p2 = [center_point_up_x, center_point_up_y]

        delta_x = center_point_up_x - center_point_dn_x
        delta_y = center_point_up_y - center_point_dn_y
        theta_radians = math.atan2(-delta_y, -delta_x)
        seam_angle = theta_radians*180/np.pi # 90 for perpendicular


        left_offset2 = 145
        left_point_up_x = right_point_up_x-left_offset2
        left_point_up_y = right_point_up_y

        p3 = [left_point_up_x,left_point_up_y]

        cv2.line(frame,(left_point_up_x,left_point_up_y),(right_point_up_x,right_point_up_y),(255,255,255),4)

        projection = ClosestPointOnLine(p2, p1, tip_pos)
        # error = np.linalg.norm(projection-p2)

        error = center_point_up_x - projection[0]

        #measured
        cv2.circle(frame, (int(projection[0]), int(projection[1])), radius=20, color=(0, 255, 255), thickness=2)
        cv2.line(frame,(tip_pos[0], tip_pos[1]), (int(projection[0]), int(projection[1])), color=(0, 255, 255), thickness=2)

        cv2.line(frame, (center_point_up_x,center_point_up_y), (int(projection[0]), int(projection[1])) , color=(255, 255, 0), thickness=1)
    
        scale = 0.129
        error = scale*error
        error_text = "error: " + str(round(abs(error),3)) + " mm"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 1
        font_thickness = 2

        if abs(error) > 1:
            cv2.putText(frame, error_text,(720, 160), font, font_size,(0,0,255),font_thickness)
            cv2.line(frame, (int((center_point_up_x+projection[0])/2), int((center_point_up_y+projection[1])/2)), (750, 180) , color=(0,0,255), thickness=2)
        elif abs(error) > 0.5:
            cv2.putText(frame, error_text,(720, 160), font, font_size,(0,255,255),font_thickness)
            cv2.line(frame, (int((center_point_up_x+projection[0])/2), int((center_point_up_y+projection[1])/2)), (750, 180) , color=(0,255,255), thickness=2)
        else:
            cv2.putText(frame, error_text,(720, 160), font, font_size,(255,255,0),font_thickness)
            cv2.line(frame, (int((center_point_up_x+projection[0])/2), int((center_point_up_y+projection[1])/2)), (750, 180) , color=(255, 255, 0), thickness=2)

    return error, seam_angle
        

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
    lines = cv2.HoughLines(edges,1,np.pi/180,threshold=10)
    
    k = np.nan
    b = np.nan
    select_one = 1

    k_list = []
    b_list = []
    d_list = []
    x_pos_list = []
    new_k_list = []
    new_b_list = []

    if lines is not None:
        # filter out the line that is too far away from point of action
        slines = select_line(new_p0, lines)
        if slines is not None:
            for line in slines:
                for rho,theta in line:
                    # n = np.array(slines).shape[0]
                    # only select the first one (strongest one) to be the edge
                    if select_one < 10:
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
                        if math.isnan(k)==False and math.isnan(b)==False:
                            k_list.append(k)
                            b_list.append(b)
                            d = np.cross(p2-p1,new_p0-p1)/np.linalg.norm(p2-p1)
                            d_list.append(abs(d))
                            # y = 0, x = -b/k
                            x_pos = -b/k
                            x_pos_list.append(x_pos)
                    select_one = select_one +1 

            d_list = np.array(d_list)
            x_pos_list = np.array(x_pos_list)
            k_list = np.array(k_list)
            b_list = np.array(b_list)

            inds = x_pos_list.argsort()
            new_k_list = np.array(k_list)[inds]
            new_b_list = np.array(b_list)[inds]
            k = new_k_list[-1]
            b = new_b_list[-1]

            x_pos = -b/k
            if not math.isnan(x_pos):
                cv2.circle(limg, (int(x_pos), 0), radius=20, color=(0, 255, 255), thickness=2)


    cv2.circle(limg, (new_p0[0],new_p0[1]), radius=3, color=(0, 255, 0), thickness=-1)
    # cv2.imshow('roi', limg)
    # cv2.imshow('edge', edges)

    return k,b

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

def plot_filter():
    """

    """
    global point_list, filter_point_list

    p1_list = np.array(point_list)[:,0]
    p2_list = np.array(point_list)[:,1]
    f_p1_list = np.array(filter_point_list)[:,0]
    f_p2_list = np.array(filter_point_list)[:,1]

    x = range(p1_list.shape[0])
    plt.ion()
    plt.show()
    plt.cla()
    
    plt.subplot(4,1,1)
    plt.plot(x, p1_list, color='r', linewidth=0.5)
    plt.title('slope: k_list')
    plt.subplot(4,1,2)
    plt.plot(x, f_p1_list, color='k', linewidth=0.5)
    plt.title('filtered_k_list')
    plt.subplot(4,1,3)
    plt.plot(x, p2_list,  color='r', linewidth=0.5)
    plt.title('y-intercept: b_list')
    plt.subplot(4,1,4)
    plt.plot(x, f_p2_list,  color='k', linewidth=0.5)
    plt.title('filtered_b_list')
        #     >>> plot(x, y, 'go--', linewidth=2, markersize=12)
        # >>> plot(x, y, color='green', marker='o', linestyle='dashed',
        # ...      linewidth=2, markersize=12)

    plt.tight_layout()
    plt.autoscale(enable=True)

    plt.draw()
    plt.pause(0.001)

def preroi_point_filter():
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
    global point_list,diff_count, filter_point_list

    # low pass filter
    a1,a2 = point_list[-1]

    new_a1 = a1
    new_a2 = a2

    # how close is this new measures to the previous one
    n = 6
    # how much the original data should be "corrected"q
    alpha = 0.1

    # k_weights=[1,1,1,2,2,2]
    k_weights = np.ones(n)
    
    if np.array(point_list).shape[0]>n:
        new_a1 = np.average(np.array(point_list)[-n:,0], weights=k_weights)
        new_a2 = np.average(np.array(point_list)[-n:,1], weights=k_weights)
        # print new_k, new_b
        if abs(new_a1-a1)>5 or abs(new_a2-a2)>5:
            if diff_count < 1:
                print("line change dtected")
                point_list[-1][0] = point_list[-2][0]
                point_list[-1][1] = point_list[-2][1]
                a1 = point_list[-1][0]
                a2 = point_list[-1][1]
                diff_count = diff_count + 1
        else:
            a1 = new_a1
            a2 = new_a2
            diff_count = 0
        

    point_list[-1][0] = alpha*a1 + (1-alpha)*point_list[-1][0]
    point_list[-1][1] = alpha*a2 + (1-alpha)*point_list[-1][1]

    # if np.array(point_list).shape[0]>n:
    #     point_list = np.array(point_list)[-n:,:]

    # filter_point_list.append([a1, a2])
    # plot_filter()

    return int(a1), int(a2)

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
    global count_nan, noedge, preroi_offset, postroi_offset, reach_first_pose, diff_count, point_list, filter_point_list, k_list, b_list

    # window offset
    k_list = []
    b_list = []
    point_list = []
    filter_point_list = []
    diff_count = 0
    count_nan = 0
    noedge = True
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

