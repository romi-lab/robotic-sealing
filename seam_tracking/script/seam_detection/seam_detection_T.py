#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script contains 2 functions:     
* convertCloudFromRosToOpen3d
* find the corresponding groove
* compute the trajectroy
* mutilayer planning

'''
from __future__ import division
import sys  
sys.path.append('/home/maggie/.local/lib/python2.7/site-packages/open3d')  
import numpy as np
from ctypes import * # convert float to uint32
import open3d as o3d
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import os
from datetime import datetime
import time
import urx
from matplotlib import pyplot as plt
import copy
from mpl_toolkits.mplot3d import Axes3D
import scipy.spatial as spatial
from sklearn.neighbors import KDTree
import vg
from scipy.spatial.transform import Rotation as R
from scipy import interpolate
import copy
import math


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# == FUNCTIONS ========================================================================================================

# Takes points in [[x1, y1, z1], [x2, y2, z2]...] Numpy Array format
def thin_line(points, point_cloud_thickness=0.5, iterations=1,sample_points=0):
    total_start_time =  time.time()
    if sample_points != 0:
        points = points[:sample_points]
    
    # Sort points into KDTree for nearest neighbors computation later
    point_tree = spatial.cKDTree(points)

    # Empty array for transformed points
    new_points = []
    # Empty array for regression lines corresponding ^^ points
    regression_lines = []
    nn_time = 0
    rl_time = 0
    prj_time = 0
    for point in point_tree.data:
        # Get list of points within specified radius {point_cloud_thickness}
        start_time = time.time()
        points_in_radius = point_tree.data[point_tree.query_ball_point(point, point_cloud_thickness)]
        nn_time += time.time()- start_time

        # Get mean of points within radius
        start_time = time.time()
        data_mean = points_in_radius.mean(axis=0)

        # Calulate 3D regression line/principal component in point form with 2 coordinates
        uu, dd, vv = np.linalg.svd(points_in_radius - data_mean)
        linepts = vv[0] * np.mgrid[-1:1:2j][:, np.newaxis]
        linepts += data_mean
        regression_lines.append(list(linepts))
        rl_time += time.time() - start_time

        # Project original point onto 3D regression line
        start_time = time.time()
        ap = point - linepts[0]
        ab = linepts[1] - linepts[0]
        point_moved = linepts[0] + np.dot(ap,ab) / np.dot(ab,ab) * ab
        prj_time += time.time()- start_time

        new_points.append(list(point_moved))
    # print("--- %s seconds to thin points ---" % (time.time() - total_start_time))
    # print(f"Finding nearest neighbors for calculating regression lines: {nn_time}")
    # print(f"Calculating regression lines: {rl_time}")
    # print(f"Projecting original points on  regression lines: {prj_time}\n")
    return np.array(new_points), regression_lines

# Sorts points outputed from thin_points()s
def sort_points(points, regression_lines, sorted_point_distance=0.01):
    sort_points_time = time.time()
    # Index of point to be sorted
    index = 0

    # sorted points array for left and right of intial point to be sorted
    sort_points_left = [points[index]]
    sort_points_right = []

    # Regression line of previously sorted point
    regression_line_prev = regression_lines[index][1] - regression_lines[index][0]

    # Sort points into KDTree for nearest neighbors computation later
    point_tree = spatial.cKDTree(points)


    # Iterative add points sequentially to the sort_points_left array
    while 1:
        # Calulate regression line vector; makes sure line vector is similar direction as previous regression line
        v = regression_lines[index][1] - regression_lines[index][0]
        if np.dot(regression_line_prev, v ) / (np.linalg.norm(regression_line_prev) * np.linalg.norm(v))  < 0:
            v = regression_lines[index][0] - regression_lines[index][1]
        regression_line_prev = v

        # Find point {distR_point} on regression line distance {sorted_point_distance} from original point 
        distR_point = points[index] + ((v / np.linalg.norm(v)) * sorted_point_distance)

        # Search nearest neighbors of distR_point within radius {sorted_point_distance / 3}
        points_in_radius = point_tree.data[point_tree.query_ball_point(distR_point, sorted_point_distance / 1.5)]
        if len(points_in_radius) < 1:
            break

        # Neighbor of distR_point with smallest angle to regression line vector is selected as next point in order
        # 
        # CAN BE OPTIMIZED
        # 
        nearest_point = points_in_radius[0]
        distR_point_vector = distR_point - points[index]
        nearest_point_vector = nearest_point - points[index]
        for x in points_in_radius: 
            x_vector = x - points[index]
            if vg.angle(distR_point_vector, x_vector) < vg.angle(distR_point_vector, nearest_point_vector):
                nearest_point_vector = nearest_point - points[index]
                nearest_point = x
        index = np.where(points == nearest_point)[0][0]

        # Add nearest point to 'sort_points_left' array
        sort_points_left.append(nearest_point)

    # Do it again but in the other direction of initial starting point 
    index = 0
    regression_line_prev = regression_lines[index][1] - regression_lines[index][0]
    while 1:
        # Calulate regression line vector; makes sure line vector is similar direction as previous regression line
        v = regression_lines[index][1] - regression_lines[index][0]
        if np.dot(regression_line_prev, v ) / (np.linalg.norm(regression_line_prev) * np.linalg.norm(v))  < 0:
            v = regression_lines[index][0] - regression_lines[index][1]
        regression_line_prev = v

        # Find point {distR_point} on regression line distance {sorted_point_distance} from original point 
        # 
        # Now vector is substracted from the point to go in other direction
        # 
        distR_point = points[index] - ((v / np.linalg.norm(v)) * sorted_point_distance)

        # Search nearest neighbors of distR_point within radius {sorted_point_distance / 3}
        points_in_radius = point_tree.data[point_tree.query_ball_point(distR_point, sorted_point_distance / 3)]
        if len(points_in_radius) < 1:
            break

        # Neighbor of distR_point with smallest angle to regression line vector is selected as next point in order
        # 
        # CAN BE OPTIMIZED
        # 
        nearest_point = points_in_radius[0]
        distR_point_vector = distR_point - points[index]
        nearest_point_vector = nearest_point - points[index]
        for x in points_in_radius: 
            x_vector = x - points[index]
            if vg.angle(distR_point_vector, x_vector) < vg.angle(distR_point_vector, nearest_point_vector):
                nearest_point_vector = nearest_point - points[index]
                nearest_point = x
        index = np.where(points == nearest_point)[0][0]

        # Add next point to 'sort_points_right' array
        sort_points_right.append(nearest_point)

    # Combine 'sort_points_right' and 'sort_points_left'
    sort_points_right = sort_points_right[::-1]
    sort_points_right.extend(sort_points_left)
    sort_points_right = np.flip(sort_points_right, 0)
    # sort_posints_right = sort_points_right[::-1]
    print("--- %s seconds to sort points ---" % (time.time() - sort_points_time))
    return np.array(sort_points_right)

def generate_new_trajectory(pcd, groove, normal):
    
    points = np.asarray(groove.points)
    
    # Thin & sort points
    thinned_points, regression_lines = thin_line(points)
    sorted_points = sort_points(thinned_points, regression_lines)

    draw = False

    if draw == True:

        # Run thinning and sorting algorithms
        # Plotting
        fig2 = plt.figure(2)
        ax3d = fig2.add_subplot(111, projection='3d')

        # Plot unordedered point cloud
        ax3d.plot(points.T[0], points.T[1], points.T[2], 'm*')

        # Plot sorted points
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], 'bo')

        # Plot line going through sorted points 
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], '-b')

        # Plot thinned points
        # ax3d.plot(thinned_points.T[0], thinned_points.T[1], thinned_points.T[2], 'go')

        fig2.show()
        plt.show()

    # trajectory = np_to_point_cloud(points, "base")

    x = sorted_points[:, 0]
    y = sorted_points[:, 1]
    z = sorted_points[:, 2]
    # print points[:]
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], s=float("inf"),full_output=1)
    # (tck, u), fp, ier, msg = interpolate.splprep([x_pos, y_pos, z_pos], s=float("inf"),full_output=1)
    # print tck
    # print u
    # line_fit = Line.best_fit(points)
    # points = line_fit.project_point(points)
    # Generate 5x points from approximated B-spline for drawing curve later
    u_fine = np.linspace(0, 1, x.size*2)

    # Evaluate points on B-spline
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    # Plot graphs
    # fig2 = plt.figure(2)
    # ax3d = fig2.add_subplot(111, projection="3d")
    # # ax3d.plot(x, y, z, "b")
    # ax3d.plot(x, y, z, "b")
    # ax3d.plot(x_fine, y_fine, z_fine, "g")
    # fig2.show()
    # plt.show()
    sorted_points = np.vstack((x_fine, y_fine, z_fine)).T
    point_size_line = sorted_points.shape[0]
    start_point = sorted_points[0]
    end_point = sorted_points[point_size_line-1]
    middle_point = (start_point + end_point)/2
    vec = end_point - middle_point
    proj = np.dot(vec, normal)*normal
    vec = vec - proj
    print np.dot(vec, normal)
    displacemnt = []

    for i in np.linspace(-1, 1, x.size):
        displacemnt.append(i*vec)
    
    # rospy.loginfo(displacemnt)
    sorted_points = np.array(np.add(displacemnt, middle_point))

    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(sorted_points)

    return trajectory_pcd

def generate_trajectory(pcd, groove):

    points = np.asarray(groove.points)
    
    # Thin & sort points
    thinned_points, regression_lines = thin_line(points)
    sorted_points = sort_points(thinned_points, regression_lines)

    draw = False

    if draw == True:

        # Run thinning and sorting algorithms
        # Plotting
        fig2 = plt.figure(2)
        ax3d = fig2.add_subplot(111, projection='3d')

        # Plot unordedered point cloud
        ax3d.plot(points.T[0], points.T[1], points.T[2], 'm*')

        # Plot sorted points
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], 'bo')

        # Plot line going through sorted points 
        ax3d.plot(sorted_points.T[0], sorted_points.T[1], sorted_points.T[2], '-b')

        # Plot thinned points
        # ax3d.plot(thinned_points.T[0], thinned_points.T[1], thinned_points.T[2], 'go')

        fig2.show()
        plt.show()

    # trajectory = np_to_point_cloud(points, "base")

    x = sorted_points[:, 0]
    y = sorted_points[:, 1]
    z = sorted_points[:, 2]
    # print points[:]
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], s=float("inf"),full_output=1)
    # (tck, u), fp, ier, msg = interpolate.splprep([x_pos, y_pos, z_pos], s=float("inf"),full_output=1)
    # print tck
    # print u
    # line_fit = Line.best_fit(points)
    # points = line_fit.project_point(points)
    # Generate 5x points from approximated B-spline for drawing curve later
    u_fine = np.linspace(0, 1, x.size*2)

    # Evaluate points on B-spline
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    # Plot graphs
    # fig2 = plt.figure(2)
    # ax3d = fig2.add_subplot(111, projection="3d")
    # # ax3d.plot(x, y, z, "b")
    # ax3d.plot(x, y, z, "b")
    # ax3d.plot(x_fine, y_fine, z_fine, "g")
    # fig2.show()
    # plt.show()
    sorted_points = np.vstack((x_fine, y_fine, z_fine)).T

    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(sorted_points)

    return trajectory_pcd

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="camera_depth_optical_frame"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

def callback_roscloud(ros_cloud):
        global received_ros_cloud

        received_ros_cloud=ros_cloud
        # rospy.loginfo("-- Received ROS PointCloud2 message.")

def transform_cam_wrt_base(pcd, T_end_effector_wrt_base):


    # T_cam_wrt_end_effector = np.array(  [[-0.01752474 ,-0.97207334, -0.23402201, 0.122350972619],
    #                                     [ 0.986549  ,  0.02123446 ,-0.16208077, -0.08164344],
    #                                     [ 0.16252373 ,-0.2337146 ,  0.95862585 ,  0.1156235],
    #                                     [ 0.          ,0.       ,   0.          ,1.        ]])



    T_cam_wrt_end_effector = np.array( [[-0.02160632 ,-0.97207334 ,-0.23368052,  0.122350972619],
                                    [ 0.98357004,  0.02123446, -0.17927374, -0.08164344],
                                    [ 0.17922931 ,-0.2337146   ,0.95564342 ,  0.1156235],
                                    [ 0.         , 0.      ,    0.         ,1.        ]])

    pcd_copy1 = copy.deepcopy(pcd).transform(T_cam_wrt_end_effector)
    pcd_copy1.paint_uniform_color([0.5, 0.5, 1])

    pcd_copy2 = copy.deepcopy(pcd_copy1).transform(T_end_effector_wrt_base)
    pcd_copy2.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([pcd, pcd_copy1, pcd_copy1, pcd_copy2])
    return pcd_copy2

def normalise_feautre(feautre_value_list):
    
    normalised_feautre_value_list = (feautre_value_list-feautre_value_list.min())/(feautre_value_list.max()-feautre_value_list.min())
    return np.array(normalised_feautre_value_list)

# find feature value list
def find_feature_value(feature, pcd, voxel_size):

    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    pc_number = np.asarray(pcd.points).shape[0]
    feautre_value_list = []
    
    n_list = np.asarray(pcd.normals)

    if feature == "asymmetry":
        neighbor = min(pc_number//100, 30)
        for index in range(pc_number):
            [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[index], neighbor)
            vector = np.mean(n_list[idx, :], axis=0)
            feature_value = np.linalg.norm(
                vector - n_list[index, :] * np.dot(vector,n_list[index, :]) / np.linalg.norm(n_list[index, :]))
            feautre_value_list.append(feature_value)

    return np.array(feautre_value_list)

def cluster_groove_from_point_cloud(pcd_selected, voxel_size, verbose=False):

    global neighbor

    # neighbor = 6*voxel_size
    labels = np.array(pcd_selected.cluster_dbscan(eps=neighbor, min_points=20, print_progress=verbose))
    max_label = labels.max()

    label, label_counts = np.unique(labels, return_counts=True)
    label_number = label[np.argsort(label_counts)[-1]]

    if label_number == -1:
        if label.shape[0]>1:
            label_number = label[np.argsort(label_counts)[-2]]
        elif label.shape[0]==1:
            # sys.exit("can not find a valid groove cluster")
            print "can not find a valid groove cluster"
    
    groove_index = np.where(labels == label_number)
    groove = pcd_selected.select_down_sample(groove_index[0])

    return groove

def save_pcd(pcd):
    now = datetime.now()
    PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"
    # write to file
    output_filename=PYTHON_FILE_PATH+"pc"+str(capture_number)+str(now)+".pcd"
    o3d.io.write_point_cloud(output_filename, pcd)
    rospy.loginfo("-- Write result point cloud to: "+output_filename)

def find_normal(trajectory, pcd):

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.003, ransac_n=20, num_iterations=100)
    [a, b, c, d] = plane_model
    plane_normal = np.array([a,b,c])
    print("\n\n\n============")
    print plane_model

    trajectory_points = np.asarray(trajectory.points)
    pcd_points = np.asarray(pcd.points)
    trajectory_number = np.array(trajectory_points).shape[0] 
    total = np.concatenate((trajectory_points, pcd_points), axis=0)
    total_pcd = o3d.geometry.PointCloud()
    total_pcd.points = o3d.utility.Vector3dVector(total)
    total_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=300))
    total_pcd.normalize_normals()
    total_pcd.orient_normals_to_align_with_direction(orientation_reference=-plane_normal)

    selected_pcd = total_pcd.select_down_sample(range(trajectory_number))
    points = np.asarray(selected_pcd.points)
    normals = np.asarray(selected_pcd.normals)
    normal = np.mean(normals, axis=0)
    return normal

def points_in_cylinder(pt1, pt2, r, query_points):
    """
    to check if a query point is in the cylinder definded as 
    @param pt1:
    @param pt2:
    @param q: query points
    @return: points inside the cylinder
    """
    vec = pt2 - pt1
    const = r * np.linalg.norm(vec)
    points_lst = []
    for q in query_points:
        flag = np.where(np.dot(q - pt1, vec) >= 0 and np.dot(q - pt2, vec) <= 0 and np.linalg.norm(np.cross(q - pt1, vec)) <= const, True, False)
        if flag:
            points_lst.append(q)
        else:
            pass
    return points_lst

def find_orientation(trajectory, pcd, groove, normal):

    global is_first_pose, is_sec_pose
    points = np.asarray(trajectory.points)

    rotvecs = []

    z_dir = normal
    pos_diff = points[1]-points[0]
    proj = np.dot(pos_diff, z_dir)*z_dir
    x_dir = pos_diff - proj
    x_dir = x_dir/np.linalg.norm(x_dir,axis=0)
    y_dir = np.cross(z_dir, x_dir)
    y_dir = y_dir/np.linalg.norm(y_dir,axis=0)
    r = R.from_dcm(np.vstack((x_dir,y_dir, z_dir)).T)
    # r = R.from_dcm(np.vstack((x_dir, y_dir, z_dir)).T)
    orientation = r.as_quat() # in the manner of 
    # euler = r.as_euler('xyz', degrees=True)
    # euler[0] = euler[0] + 180
    # new_r = R.from_euler('xyz', euler, degrees=True)
    rotvec = r.as_rotvec()
    # rotvec[1] = rotvec[1]+np.pi

    for i in range(np.array(points).shape[0]):

        if is_first_pose:

            #publish to tip_pose
            first_pose = Pose()
            #tip position
            first_pose.position.x = points[i][0]
            first_pose.position.y = points[i][1]
            first_pose.position.z = points[i][2]
            #tip orientation
            first_pose.orientation.x = orientation[0]
            first_pose.orientation.y = orientation[1]
            first_pose.orientation.z = orientation[2]
            first_pose.orientation.w = orientation[3]
            pub_first_pose.publish(first_pose)
            print("first pose published")
            is_first_pose = False
            is_sec_pose = True
            #publish pen tip pose trajectory
        
        if is_first_pose == False and is_sec_pose == True:

            first_point = points[i-1][0:3]
            sec_point = points[i][0:3]
            velocity_vec = sec_point-first_point
            print("velocity vector published")
            print(velocity_vec)

            vel_vec_point = Point()
            vel_vec_point.x = -velocity_vec[0]
            vel_vec_point.y = -velocity_vec[1]
            vel_vec_point.z = -velocity_vec[2]

            pub_vel_vec.publish(vel_vec_point)

            is_sec_pose=False

        #publish as marker array to check the sequence, id = base for vosualization only
        marker = Marker() 
        marker.header.frame_id = 'base'
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.pose.position.x = points[i][0]
        marker.pose.position.y = points[i][1]
        marker.pose.position.z = points[i][2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.text = str(i)
        markerArray.markers.append(marker)

        rotvecs.append(rotvec)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    pub_marker.publish(markerArray)

    ur_poses = np.hstack((points, np.array(rotvecs)))

    return ur_poses

def trajectory_execution(pose_list):
    print "\nPress `Enter` to execute or q to quit: "
    if not raw_input() == 'q':
        # tcp_torch = [-0.00072, 0.05553, 0.2312, -0.8504775921315857, -0.02340453557068149, -0.015929517346989313]
        tcp_torch = [-0.0002, -0.08916, 0.32202, 0, 0, 0]
        robot.set_tcp(tcp_torch)
        time.sleep(0.2) #pause is essentail for tcp to take effect, min time is 0.1s

        approah = [0,0,0,0,0,0]
        approah[0] = pose_list[0][0]
        # approah[1] = pose_list[0][1] + 0.05
        approah[1] = pose_list[0][1]
        approah[2] = pose_list[0][2] + 0.05
        approah[3] = pose_list[0][3]
        approah[4] = pose_list[0][4]
        approah[5] = pose_list[0][5]
        first_pose = copy.deepcopy(pose_list[0])
        robot.movel(approah, acc=0.1, vel=0.1, wait=True)
        robot.movel(first_pose, acc=0.1, vel=0.1, wait=True)

        start = Point()
        start.x = 1
        start.y = 0
        start.z = 0
        pub_start_process.publish(start)

        raw_input("press any to contiue")

        
        # time.sleep(0.2)
        robot.movel(pose_list[1], acc=0.003, vel=0.003, wait=True)
        # robot.movel(pose_list[2], acc=0.1, vel=0.1, wait=True)
        robot.set_digital_out(0,True)
        robot.movels(pose_list[2:], acc=0.003, vel=0.003, wait=True)

        start = Point()
        start.x = -1
        start.y = 0
        start.z = 0
        pub_start_process.publish(start)

        robot.set_digital_out(0,False)
        # raw_input("Press any to continue")
        robot.translate_tool((0, 0, -0.05), vel=0.1, acc=0.1, wait=True)

def mutilayer(poses):
    
    z_height=0.0006 #m
    y_height=0.002
    r_origin = R.from_rotvec(poses[0][3:])
    Rot_matrix = r_origin.as_dcm()
    new_y = Rot_matrix[:,1]
    new_z = Rot_matrix[:,2]
    z_offset = -new_z*z_height
    y_offset = -new_y*y_height

    angle = math.atan2(z_height, y_height)
    print angle
    left_angle = angle/2-np.pi/4
    poses_copy_left = copy.deepcopy(poses)
    poses_copy_right = copy.deepcopy(poses)

    left_poses = []
    for left_ur_pose in poses_copy_left:
        left_ur_pose[0] = left_ur_pose[0] + z_offset[0] + y_offset[0]
        left_ur_pose[1] = left_ur_pose[1] + z_offset[1] + y_offset[1]
        left_ur_pose[2] = left_ur_pose[2] + z_offset[2] + y_offset[2]
        r_orien_left = R.from_euler('x', left_angle, degrees=False)
        r_left = r_orien_left*r_origin
        left_orientation = r_left.as_rotvec()
        left_ur_pose[3:]=left_orientation
        left_poses.append(left_ur_pose)      
    
    right_angle = -left_angle

    right_poses = []
    for right_ur_pose in poses_copy_right:
        right_ur_pose[0] = right_ur_pose[0] + z_offset[0] - y_offset[0]
        right_ur_pose[1] = right_ur_pose[1] + z_offset[1] - y_offset[1]
        right_ur_pose[2] = right_ur_pose[2] + z_offset[2] - y_offset[2]
        r_orien_right = R.from_euler('x', right_angle, degrees=False)
        r_right = r_orien_right*r_origin
        right_orientation = r_right.as_rotvec()
        right_ur_pose[3:]=right_orientation
        right_poses.append(right_ur_pose)     
    
    return left_poses, right_poses

def uplift_z(ur_poses):

    r = R.from_rotvec(ur_poses[0][3:])
    Rot_matrix = r.as_dcm()
    new_z = Rot_matrix[:,2]
    new_y = Rot_matrix[:,1]
    new_x = Rot_matrix[:,0]
    #the length of pen 0.22m, cube dim: 0.02*0.02 unit:m
    # pen_length = 0.19 #planer
    offset_z = 0.002
    offset_y = 0.00
    offset_x = 0.000
    # offset = 0.22 for setting camera as tcp
    displacement_z = offset_z*new_z
    displacement_y = offset_y*new_y
    displacement_x = offset_x*new_x
    new_ur_poses = []
    for urpose in ur_poses:

        urpose[0] = urpose[0] + displacement_z[0] + displacement_y[0] + displacement_x[0] 
        urpose[1] = urpose[1] + displacement_z[1] + displacement_y[1] + displacement_x[1]
        urpose[2] = urpose[2] + displacement_z[2] + displacement_y[2] + displacement_x[2]
        # sealing on two layer
        # urpose[1] = urpose[1] + displacement_z[1] + displacement_y[1] + displacement_x[1] - 0.001
        # urpose[2] = urpose[2] + displacement_z[2] + displacement_y[2] + displacement_x[2] - 0.009
        new_ur_poses.append(urpose)
    return new_ur_poses

def get3Points():

    pass

def detect_groove_workflow(pcd, transfromation_end_to_base, detect_feature="asymmetry" , show_groove=False, publish=True, save_data=False):

     # 2.downsample of point cloud

    global max_dis, total_time, voxel_size, delete_percentage

    # raw_input("press to continue")

    original_pcd = pcd

    voxel_size = 0.005
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    pcd_points = np.asarray(pcd.points)
    pcd.clear()
    pcd_points = pcd_points[pcd_points[:,2]<max_dis]
    pcd.points = o3d.utility.Vector3dVector(pcd_points)
    
    pcd.remove_none_finite_points()
    # pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    # pcd.remove_radius_outlier(nb_points=20, radius = 4*voxel_size)
    pc_number = np.asarray(pcd.points).shape[0]
    rospy.loginfo("Total number of pc {}".format(pc_number))


    # 3.estimate normal toward cam location and normalise it
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
         radius=0.01, max_nn=30))
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location(camera_location=[0., 0., 0.])

    # # 4.use different geometry features to find groove
    # feautre_value_list = find_feature_value(detect_feature, pcd, voxel_size)
    # normalised_feautre_value_list = normalise_feautre(feautre_value_list)
    #
    # # 5.delete low value points and cluster
    # delete_points = int(pc_number*delete_percentage)
    # pcd_selected = pcd.select_down_sample(np.argsort(normalised_feautre_value_list)[delete_points:])
    # groove = cluster_groove_from_point_cloud(pcd_selected, voxel_size)

    groove_t = time.time()
    rospy.loginfo("Runtime of groove detection is {}".format(groove_t-start))

    # Start to label the three points
    pcd.paint_uniform_color([0.8, 0.8, 0.8])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    print(vis.get_picked_points())
    pick_idx_lst = vis.get_picked_points()
    print("The indexes of the selected points are: \n {} ".format(pick_idx_lst))

    extend_ratio = 0.11
    extend_ratio = 0
    r = voxel_size * 18
    start_p = np.asarray(pcd.points)[pick_idx_lst[0]]
    corner_p_true = np.asarray(pcd.points)[pick_idx_lst[1]]
    end_p = np.asarray(pcd.points)[pick_idx_lst[2]]
    corner_p = corner_p_true + (corner_p_true - start_p) * extend_ratio
    ROI_points_lst1 = points_in_cylinder(start_p, corner_p, r, np.asarray(pcd.points))
    corner_p = corner_p_true + (corner_p_true - end_p) * extend_ratio
    ROI_points_lst2 = points_in_cylinder(end_p, corner_p, r, np.asarray(pcd.points))
    ROI_points_lst = np.concatenate((ROI_points_lst1, ROI_points_lst2), axis=0)

    ROI_pcd = o3d.geometry.PointCloud()
    ROI_pcd.points = o3d.utility.Vector3dVector(ROI_points_lst)
    ROI_pcd.paint_uniform_color([1, 0.5, .5])
    pcd.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([pcd, ROI_pcd])

    groove1 = o3d.geometry.PointCloud()
    groove1.points = o3d.utility.Vector3dVector(ROI_points_lst1)
    groove2 = o3d.geometry.PointCloud()
    groove2.points = o3d.utility.Vector3dVector(ROI_points_lst2)

    pcd = transform_cam_wrt_base(pcd, transfromation_end_to_base)
    groove1 = transform_cam_wrt_base(groove1, transfromation_end_to_base)
    groove2 = transform_cam_wrt_base(groove2, transfromation_end_to_base)

    trajectory1 = generate_trajectory(pcd, groove1)
    trajectory2 = generate_trajectory(pcd, groove2)

    normal1 = find_normal(trajectory1, pcd)
    normal2 = find_normal(trajectory2, pcd)

    points1 = np.asarray(trajectory1.points)
    point_size_line1 = points1.shape[0]
    start_point1 = points1[0]
    end_point1 = points1[point_size_line1-1]

    points2 = np.asarray(trajectory2.points)
    point_size_line2 = points2.shape[0]
    start_point2 = points2[0]
    end_point2 = points2[point_size_line2 - 1]

    refined_groove1 = points_in_cylinder(start_point1, end_point1, voxel_size*5, np.asarray(groove1.points))
    refined_groove_pcd1 = o3d.geometry.PointCloud()
    refined_groove_pcd1.points = o3d.utility.Vector3dVector(refined_groove1)
    refined_groove_pcd1.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    new_trajectory1 = generate_new_trajectory(pcd, refined_groove_pcd1, normal1)
    ur_poses1 = find_orientation(new_trajectory1, pcd, refined_groove_pcd1, normal1)
    groove1 = refined_groove_pcd1

    refined_groove2 = points_in_cylinder(start_point2, end_point2, voxel_size * 5, np.asarray(groove2.points))
    refined_groove_pcd2 = o3d.geometry.PointCloud()
    refined_groove_pcd2.points = o3d.utility.Vector3dVector(refined_groove2)
    refined_groove_pcd2.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    new_trajectory2 = generate_new_trajectory(pcd, refined_groove_pcd2, normal2)
    ur_poses2 = find_orientation(new_trajectory2, pcd, refined_groove_pcd2, normal2)
    groove2 = refined_groove_pcd2


    traj_t = time.time()
    rospy.loginfo("Runtime of trajectory is {}".format(traj_t-groove_t))

    if publish:
        # the point cloud is wrt base link (correct)
        # however for visalization purpose, we need to set it to base
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
        rviz_cloud = convertCloudFromOpen3dToRos(pcd, frame_id="base")
        pub_pc.publish(rviz_cloud)

        groove1.paint_uniform_color([1, 0, 0])
        rviz_groove1 = convertCloudFromOpen3dToRos(groove1, frame_id="base")
        pub_groove.publish(rviz_groove1)

        groove2.paint_uniform_color([0, 0, 1])
        rviz_groove2 = convertCloudFromOpen3dToRos(groove2, frame_id="base")
        pub_groove.publish(rviz_groove2)

        trajectory1.paint_uniform_color([0, 1, 0])
        rviz_trajectory1 = convertCloudFromOpen3dToRos(trajectory1, frame_id="base")
        pub_trajectory.publish(rviz_trajectory1)

        trajectory2.paint_uniform_color([0, 1, 0])
        rviz_trajectory2 = convertCloudFromOpen3dToRos(trajectory2, frame_id="base")
        pub_trajectory.publish(rviz_trajectory2)

        rospy.loginfo("Conversion and publish success ...\n")
        # rospy.sleep(1)

    if show_groove:
        o3d.visualization.draw_geometries([pcd, groove, trajectory1, trajectory2])
        # o3d.visualization.draw_geometries([pcd])
    if save_data:
        save_pcd(original_pcd)
    end = time.time()
    rospy.loginfo("Runtime of process is {}".format(end-start))
    total_time.append(end-start)
    rospy.loginfo("Total process {}".format(np.array(total_time).shape[0]))
    rospy.loginfo("Average Runtime of process is {}".format(np.mean(total_time)))
    return [ur_poses1, ur_poses2]

def rotate_pose_for_sealing(poses):

    rotate_R = R.from_euler('y', -35, degrees=True)
    rotate_pose = []

    for pose in poses:

        r = R.from_rotvec(pose[3:])
        new_r = r*rotate_R
        new_rot_vec = new_r.as_rotvec()
        pose[3:]=new_rot_vec
        rotate_pose.append(pose)
    
    return rotate_pose

def publish_trajectory(poses):

    PoseList_tip_rviz = PoseArray()

    for pose in poses:

        r = R.from_rotvec(pose[3:])
        orientation = r.as_quat()

        #publish to tip_pose
        tip_pose_rviz = Pose()
        #tip position
        tip_pose_rviz.position.x = pose[0]
        tip_pose_rviz.position.y = pose[1]
        tip_pose_rviz.position.z = pose[2]
        #tip orientation
        tip_pose_rviz.orientation.x = orientation[0]
        tip_pose_rviz.orientation.y = orientation[1]
        tip_pose_rviz.orientation.z = orientation[2]
        tip_pose_rviz.orientation.w = orientation[3]
        #publish pen tip pose trajectory
        PoseList_tip_rviz.poses.append(tip_pose_rviz)

    PoseList_tip_rviz.header.frame_id = 'base'
    PoseList_tip_rviz.header.stamp = rospy.Time.now()
    pub_poses.publish(PoseList_tip_rviz)


if __name__ == "__main__":
       
    rospy.init_node('welding_demo', anonymous=True)

    robot = urx.Robot("192.168.0.2")

    # -- Set subscriber
    global received_ros_cloud, delete_percentage, max_dis, total_time, voxel_size, neighbor, is_first_pose, is_sec_pose
    
    is_first_pose = True
    is_sec_pose = False
    voxel_size = 0.005
    neighbor = 5*voxel_size
    delete_percentage = 0.95

    received_ros_cloud = None
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_roscloud, queue_size=1)  

    # -- Set publisher
    pub_groove = rospy.Publisher("groove_points", PointCloud2, queue_size=1)
    pub_pc = rospy.Publisher("downsampled_points", PointCloud2, queue_size=1)
    pub_trajectory = rospy.Publisher("trajectory", PointCloud2, queue_size=1)
    pub_poses = rospy.Publisher("pose_trajectory", PoseArray, queue_size=1)
    pub_marker = rospy.Publisher('sequences', MarkerArray, queue_size=1)
    pub_first_pose = rospy.Publisher('first_pose', Pose, queue_size=1)
    pub_vel_vec = rospy.Publisher('vel_vector', Point, queue_size=1)
    pub_start_process = rospy.Publisher('start_process', Point, queue_size=1)

    capture_number = 0 

    horizontal_plane = [-1.3588979879962366, -1.0134013334857386, -2.2536891142474573, -0.17619353929628545, 1.302361011505127, -1.509554688130514]

    # parameter 
    # max_dis = 0.4
    # offset_z = -0.003
    # offset_y = -0.003

    normal_plane = [-1.5366695562945765, -1.6485264937030237, -1.1686652342425745, -1.7946108023272913, 1.4708768129348755, -1.4940932432757776]

    vertical_plane = [-1.340297047291891, -1.1544345060931605, -2.4866958300219935, 0.49034059047698975, 1.2755035161972046, -1.567136589680807]

    # offset_z = -0.005
    # offset_y = -0.003
    # offset_x = 0.005

    cube = [-1.32884389558901, -1.4910205046283167, -2.6015103499041956, 1.2944217920303345, 1.2764376401901245, -1.646374527608053]
    beam =  [-1.733206574116842, -1.075972859059469, -2.091264073048727, -0.379343334828512, 2.190109968185425, -0.44370919862856084]
    I_beam =  [-0.9549530188189905, -1.5682643095599573, -2.155350987111227, 0.12100887298583984, 0.1719762533903122, 0.5123952627182007]

    sealing_two_plane =  [-1.3595932165728968, -1.536405388508932, -1.3151958624469202, -1.7422354857074183, 1.4140154123306274, 0.17579922080039978]

    t_cube = []

    startj =  normal_plane
    execution = True
    max_dis = 0.7
    mutilayer_exe = False
    total_time = []
    sealing = True
    # sealing = False   

    while not rospy.is_shutdown():
        
        robot.movej(startj, acc=0.8, vel=0.4, wait=True)
        time.sleep(0.5)

        if execution == True:
            is_first_pose = True
            is_sec_pose = False
            capture = raw_input("\n\nstart capture point cloud, q to quit: ")
            # capture = ''
            if capture == 'q':
                break
            else:
                robot.movej(startj, acc=0.8, vel=0.4, wait=True)
                # convert it back to open3d, and draw
                print("starting, please don't move the workpiece")
                time.sleep(0.5)
                capture_number += 1
                robot.set_tcp((0, 0, 0, 0, 0, 0))
                time.sleep(0.3)
                if not received_ros_cloud is None:

                    print("\n=========================start seam detection====================")
                    start = time.time() #start counting
                    received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
                    T_end_effector_wrt_base = robot.get_pose()
                    markerArray = MarkerArray()
                    PoseList_tip_rviz = PoseArray()

                    ur_poses = detect_groove_workflow(received_open3d_cloud, T_end_effector_wrt_base.array, detect_feature="asymmetry", show_groove=False)
                    ur_poses1 = ur_poses[0]
                    ur_poses1 = uplift_z(ur_poses1)

                    if sealing:
                        ur_poses1 = rotate_pose_for_sealing(ur_poses1)

                    publish_trajectory(ur_poses1)
                    trajectory_execution(ur_poses1)

                    if mutilayer_exe:
                        left_poses, right_poses = mutilayer(ur_poses)
                        publish_trajectory(left_poses)
                        trajectory_execution(left_poses)
                        publish_trajectory(right_poses)
                        trajectory_execution(right_poses)

                    rospy.loginfo("-- Finish display. ...\n")
        else:
            is_first_pose = True
            is_sec_pose = False
            robot.set_tcp((0, 0, 0, 0, 0, 0))
            # time.sleep(1)
            if not received_ros_cloud is None:

                print("\n=========================start seam detection====================")
                # capture = raw_input("\n\nstart capture point cloud, q to quit: ")
                start = time.time() #start counting
                received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
                T_end_effector_wrt_base = robot.get_pose()
                markerArray = MarkerArray()
                PoseList_tip_rviz = PoseArray()

                ur_poses = detect_groove_workflow(received_open3d_cloud, T_end_effector_wrt_base.array, detect_feature="asymmetry", show_groove=False)
                
                rospy.loginfo("-- Finish display. ...\n")
    robot.stop()
    robot.close()
    rospy.signal_shutdown("Finished shutting down")
