# Created by Jeffery at 22:28 1/21/21 using PyCharm
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math

img = cv2.imread('./color_00002.png')
# img = cv2.imread('./color_00006.png')
# edges = cv2.Canny(img, 20,100) # for T cubes
# edges = cv2.Canny(img, 20, 100)

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel_size = 3
blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)
edges = cv2.Canny(blur_gray, 20, 100)
# cv2.Canny(gaus, 50, 150, apertureSize=3)
# cv2.imshow("houghline", edges)
# cv2.waitKey()

rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 15  #
min_line_length = 50  # minimum number of pixels making up a line
max_line_gap = 20  #
line_image = np.copy(img) * 0  # creating a blank to draw lines on
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

def get_dist(point, line_point1, line_point2):
    # the distance between the point and a line
    if line_point1 == line_point2:
        point_array = np.array(point )
        point1_array = np.array(line_point1)
        return np.linalg.norm(point_array -point1_array )
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A**2 + B**2))
    return distance

print(edges.shape)

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()

for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),1)
lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

def getPointInLine(px, py, x1, y1, x2, y2):
    # get the nearest point to a query point in the line
    k = 1. * (y2-y1)/(x2-x1)
    x = 1. *(k*k*x1 + k * (py - y1) + px) / (k*k + 1)
    y = 1.0 * k * (x - x1) + y1
    return [int(x), int(y)]

point_lst = []
line_lst = []
def draw_circle(event,x,y, flags,param):
    global mouseX,mouseY
    global point_lst
    global lines, line_lst
    if event == cv2.EVENT_LBUTTONDOWN:
        # cv2.circle(img, (x,y),1,(0,0,255),-1)
        mouseX,mouseY = x,y
        xy = "%d,%d" % (x, y)
        cv2.circle(lines_edges, (x, y), 5, (127, 255, 0), thickness=1)
        cv2.putText(lines_edges, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (127, 255, 0), thickness=2)
        dist_list = [get_dist([x, y], [line[0][0], line[0][1]], [line[0][2], line[0][3]]) for line in lines]
        min_idx = dist_list.index(min(dist_list))
        print(min_idx) # the index of the nearest line
        x1, y1, x2, y2 = lines[min_idx][0][0], lines[min_idx][0][1],  lines[min_idx][0][2], lines[min_idx][0][3]
        print(x1, y1, x2, y2)
        # cv2.line(lines_edges, (x1, y1), (x2, y2), (127, 255, 0), 5)
        point_lst.append([x, y])
        if line_lst and min([np.sqrt((x1-i[0])*(x1-i[0])+(y1-i[1])*(y1-i[1])) for i in line_lst]) <= 10:
            corner = tuple([x1, y1])
            # p1 = tuple(getPointInLine(point_lst[0][0], point_lst[0][1], x1, y1, x2, y2))
            # p2 = tuple(getPointInLine(point_lst[1][0], point_lst[1][1], x1, y1, x2, y2))
            p1 = tuple(point_lst[0])
            p2 = tuple(point_lst[1])
            cv2.line(lines_edges, p1, corner, (0, 255, 0), 2)
            cv2.line(lines_edges, corner, p2, (0, 255, 0), 2)
            point_lst.append(corner)
        elif line_lst and min([np.sqrt((x2-i[0])*(x2-i[0])+(y2-i[1])*(y2-i[1])) for i in line_lst]) <= 10:
            corner = tuple([x2, y2])
            # p1 = tuple(getPointInLine(point_lst[0][0], point_lst[0][1], x1, y1, x2, y2))
            # p2 = tuple(getPointInLine(point_lst[1][0], point_lst[1][1], x1, y1, x2, y2))
            p1 = tuple(point_lst[0])
            p2 = tuple(point_lst[1])
            cv2.line(lines_edges, p1, corner, (0, 255, 0), 2)
            cv2.line(lines_edges, corner, p2, (0, 255, 0), 2)
            point_lst.append(corner)
        else:
            line_lst.append([x1, y1])
            line_lst.append([x2, y2])
        print(point_lst)
        print(line_lst)



cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
global mouseX,mouseY

while(1):
    cv2.imshow('image',lines_edges)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):

        print(mouseX,mouseY)
