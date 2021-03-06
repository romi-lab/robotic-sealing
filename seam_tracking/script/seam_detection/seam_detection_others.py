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
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
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
def thin_line(points, point_cloud_thickness=0.02, iterations=1,sample_points=0):
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
def sort_points(points, regression_lines, sorted_point_distance=0.02):
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
        points_in_radius = point_tree.data[point_tree.query_ball_point(distR_point, sorted_point_distance / 1.2)]
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

def generate_trajectory(pcd, groove):

    points = np.asarray(groove.points)
    
    # Thin & sort points
    thinned_points, regression_lines = thin_line(points)
    sorted_points = sort_points(thinned_points, regression_lines)

    x = sorted_points[:, 0]
    y = sorted_points[:, 1]
    z = sorted_points[:, 2]
    # # print points[:]
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], k=3, s=5, full_output=1)

    u_fine = np.linspace(0, 1, x.size*2)

    # # Evaluate points on B-spline
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    sorted_points = np.vstack((x_fine, y_fine, z_fine)).T

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
    
    
    T_cam_wrt_end_effector = np.array(  [[ 0.04708804, -0.97102151, -0.23430736,  0.128350972619 ],
                                        [ 0.99791492,  0.05609536, -0.03192367,  -0.128848698927 ],
                                        [ 0.04414212, -0.23231558,  0.97163828, 0.0962031389762 ],
                                        [ 0., 0., 0., 1. ]])

    T_cam_wrt_end_effector = np.array(  [[ 0.00211918, -0.97207334, -0.23466769,  0.122350972619],
                                        [ 0.9966516 ,  0.02123446 ,-0.07896004, -0.08004344],
                                        [ 0.08173799, -0.2337146 ,  0.96886345 , 0.1156235],
                                        [ 0.      ,    0.         , 0.      ,    1.        ]])


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

def cluster_groove_from_point_cloud(pcd_selected, voxel_size, verbose=False, visualiza_process= True):

    neighbor = 4*voxel_size
    labels = np.array(pcd_selected.cluster_dbscan(eps=neighbor, min_points=10, print_progress=verbose))
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

    if visualiza_process == True:
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd_selected.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([pcd_selected])

    return groove

def detect_groove_workflow(pcd, transfromation_end_to_base, detect_feature="asymmetry" , visualiza_process= True, show_groove=True, publish=True, save_data=False):

     # 2.downsample of point cloud

    global max_dis, total_time

    voxel_size = 0.001
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    if visualiza_process:
        o3d.visualization.draw_geometries([pcd])

    pcd_points = np.asarray(pcd.points)
    pcd.clear()
    pcd_points = pcd_points[pcd_points[:,2]<max_dis]
    pcd.points = o3d.utility.Vector3dVector(pcd_points)
    

    
    pcd.remove_none_finite_points()
    pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    pcd.remove_radius_outlier(nb_points=20, radius = 4*voxel_size)
    pc_number = np.asarray(pcd.points).shape[0]
    rospy.loginfo("Total number of pc {}".format(pc_number))


    # 3.estimate normal toward cam location and normalise it
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
         radius=0.01, max_nn=30))
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location(camera_location=[0., 0., 0.])

    # 4.use different geometry features to find groove
    feautre_value_list = find_feature_value(detect_feature, pcd, voxel_size)
    normalised_feautre_value_list = normalise_feautre(feautre_value_list)
    

    if visualiza_process:
        color_map = "rainbow"  #OrRd, binary, tab20, rainbow, hsv, plasma, gist_rainbow, jet
        colors = plt.get_cmap(color_map)(normalised_feautre_value_list)
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([pcd])

    # 5.delete low value points and cluster
    delete_points = int(pc_number*0.93)
    pcd_selected = pcd.select_down_sample(np.argsort(normalised_feautre_value_list)[delete_points:])
    

    if visualiza_process:
        o3d.visualization.draw_geometries([pcd_selected])

    groove = cluster_groove_from_point_cloud(pcd_selected, voxel_size)
    groove_t = time.time()
    rospy.loginfo("Runtime of groove detection is {}".format(groove_t-start))
    # groove.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.5)

    # index = np.where(np.isin(np.asarray(pcd.points), np.asarray(groove.points))==True)
    # np.asarray(pcd.colors)[index[0],:] = [1,0,0]
    # print pcd 
    # print groove
    pcd = transform_cam_wrt_base(pcd, transfromation_end_to_base)
    groove = transform_cam_wrt_base(groove, transfromation_end_to_base)

    trajectory = generate_trajectory(pcd, groove)
    ur_poses = find_orientation(trajectory, pcd)

    traj_t = time.time()
    rospy.loginfo("Runtime of trajectory is {}".format(traj_t-groove_t))

    if publish:
        # the point cloud is wrt base link (correct)
        # however for visalization purpose, we need to set it to base
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
        rviz_cloud = convertCloudFromOpen3dToRos(pcd, frame_id="base")
        pub_pc.publish(rviz_cloud)

        groove.paint_uniform_color([1, 0, 0])
        rviz_groove = convertCloudFromOpen3dToRos(groove, frame_id="base")
        pub_groove.publish(rviz_groove)

        trajectory.paint_uniform_color([0, 1, 0])
        rviz_trajectory = convertCloudFromOpen3dToRos(trajectory, frame_id="base")
        pub_trajectory.publish(rviz_trajectory)

        rospy.loginfo("Conversion and publish success ...\n")
        # rospy.sleep(1)

    if show_groove:
        o3d.visualization.draw_geometries([pcd, groove, trajectory])
        # o3d.visualization.draw_geometries([pcd])

    if save_data:
        save_pcd(pcd)
    
    end = time.time()
    rospy.loginfo("Runtime of process is {}".format(end-start))
    total_time.append(end-start)
    rospy.loginfo("Total process {}".format(np.array(total_time).shape[0]))
    rospy.loginfo("Average Runtime of process is {}".format(np.mean(total_time)))

    return ur_poses

def find_orientation(trajectory, pcd):
    
    trajectory = np.asarray(trajectory.points)
    pcd = np.asarray(pcd.points)
    trajectory_number = np.array(trajectory).shape[0]
    total = np.concatenate((trajectory,pcd), axis=0)
    total_pcd = o3d.geometry.PointCloud()
    total_pcd.points = o3d.utility.Vector3dVector(total)
    total_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=100))
    total_pcd.normalize_normals()
    total_pcd.orient_normals_towards_camera_location(camera_location=[0, 0, 0])

    selected_pcd = total_pcd.select_down_sample(range(trajectory_number))
    points = np.asarray(selected_pcd.points)
    normals = -np.asarray(selected_pcd.normals)
    rotvecs = []
    

    for i in range(np.array(points).shape[0]):
        
        pos_diff = 0
        z_dir = normals[i]
        #find x direction based on point   
        if i == 0:
            pos_diff = points[i+1]-points[i]
            z_dir = normals[i+1]
        elif i == (np.array(points).shape[0]-1):
            pos_diff = points[i]-points[i-1]
            z_dir = normals[i-1]
        else:
            pos_diff = points[i+1]-points[i]
            z_dir = normals[i]

        proj = np.dot(pos_diff, z_dir)*z_dir
        x_dir = pos_diff - proj
        x_dir = x_dir/np.linalg.norm(x_dir,axis=0)
        y_dir = np.cross(z_dir, x_dir)
        y_dir = y_dir/np.linalg.norm(y_dir,axis=0)
        r = R.from_dcm(np.vstack((x_dir, y_dir, z_dir)).T)
        orientation = r.as_quat() # in the manner of 
        rotvec = r.as_rotvec()
        rotvecs.append(rotvec)

        #publish to tip_pose
        tip_pose_rviz = Pose()
        #tip position
        tip_pose_rviz.position.x = points[i][0]
        tip_pose_rviz.position.y = points[i][1]
        tip_pose_rviz.position.z = points[i][2]
        #tip orientation
        tip_pose_rviz.orientation.x = orientation[0]
        tip_pose_rviz.orientation.y = orientation[1]
        tip_pose_rviz.orientation.z = orientation[2]
        tip_pose_rviz.orientation.w = orientation[3]
        #publish pen tip pose trajectory
        PoseList_tip_rviz.poses.append(tip_pose_rviz)

        #publish as marker array to check the sequence
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
    
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    pub_marker.publish(markerArray)

    PoseList_tip_rviz.header.frame_id = 'base'
    PoseList_tip_rviz.header.stamp = rospy.Time.now()
    pub_poses.publish(PoseList_tip_rviz)
    ur_poses = np.hstack((points, np.array(rotvecs)))

    return ur_poses

def trajectory_execution(pose_list):
    print "\nPress `Enter` to execute or q to quit: "
    if not raw_input() == 'q':
        # tcp_torch = [-0.00072, 0.05553, 0.2312, -0.8504775921315857, -0.02340453557068149, -0.015929517346989313]
        tcp_torch = [-0.0002, -0.09302, 0.31, 0, 0, 0]
        robot.set_tcp(tcp_torch)
        time.sleep(0.5) #pause is essentail for tcp to take effect, min time is 0.1s

        robot.movel(pose_list[0], acc=0.1, vel=0.1, wait=True)
        robot.movels(pose_list, acc=0.02, vel=0.02, wait=True)
        raw_input("Press any to continue")
        robot.translate_tool((0, 0, -0.08), vel=0.1, acc=0.1, wait=True)

def uplift_z(ur_poses):
    # print ur_poses[:,2]
    # print np.mean(ur_poses[:,2])
    new_ur_poses = []
    for urpose in ur_poses:
        r = R.from_rotvec(urpose[3:])
        Rot_matrix = r.as_dcm()
        new_z = Rot_matrix[:,2]
        new_y = Rot_matrix[:,1]
        #the length of pen 0.22m, cube dim: 0.02*0.02 unit:m
        # pen_length = 0.19 #planer
        offset_z = -0.015
        offset_y = -0.01
        # offset = 0.22 for setting camera as tcp
        displacement_z = offset_z*new_z
        displacement_y = offset_y*new_y
        urpose[0] = urpose[0] + displacement_z[0] + displacement_y[0]
        urpose[1] = urpose[1] + displacement_z[1] + displacement_y[1]
        urpose[2] = urpose[2] + displacement_z[2] + displacement_y[2]
        new_ur_poses.append(urpose)
    return new_ur_poses

def mutilayer(poses):
    
    z_height=-0.004 #m
    y_height=-0.006
    r_origin = R.from_rotvec(poses[0][3:])
    Rot_matrix = r_origin.as_dcm()
    new_y = Rot_matrix[:,1]
    new_z = Rot_matrix[:,2]
    z_offset = new_z*z_height
    y_offset = new_y*y_height

    angle = math.atan(y_height/z_height)
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

if __name__ == "__main__":
       
    rospy.init_node('welding_demo', anonymous=True)

    robot = urx.Robot("192.168.0.2")

    # -- Set subscriber
    global received_ros_cloud, max_dis, total_time, max_dis
    received_ros_cloud = None
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_roscloud, queue_size=1)  

    # -- Set publisher
    pub_groove = rospy.Publisher("groove_points", PointCloud2, queue_size=1)
    pub_pc = rospy.Publisher("downsampled_points", PointCloud2, queue_size=1)
    pub_trajectory = rospy.Publisher("trajectory", PointCloud2, queue_size=1)
    pub_poses = rospy.Publisher("pose_trajectory", PoseArray, queue_size=1)
    pub_marker = rospy.Publisher('sequences', MarkerArray, queue_size=1)

    capture_number = 0 


    tube = [-1.1437214056598108, -1.3365314642535608, -1.840600315724508, 0.20955729484558105, 1.1416419744491577, -1.66575795808901]
    intersect = [-1.7001226584063929, -1.2195118109332483, -1.8784139792071741, -0.799168888722555, 1.625313401222229, -1.5555179754840296]
    cube_intersect = [-1.975440804158346, -1.4402702490436, -1.8757017294513147, -0.4285944143878382, 1.8892546892166138, -1.6770418326007288]
    startj = cube_intersect
    execution = True
    max_dis = 0.35
    mutilayer_exe = False
    total_time = []


    while not rospy.is_shutdown():
        
        robot.movej(startj, acc=0.8, vel=0.4, wait=True)
        time.sleep(1)

        if execution == True:
            capture = raw_input("\n\nstart capture point cloud, q to quit: ")
            # capture = ''
            if capture == 'q':
                robot.close()
                break
            else:
                # convert it back to open3d, and draw
                print("starting, please don't move the workpiece")
                time.sleep(2)
                capture_number += 1
                robot.set_tcp((0, 0, 0, 0, 0, 0))
                time.sleep(1)
                if not received_ros_cloud is None:

                    print("\n=========================start seam detection====================")
                    start = time.time() #start counting
                    received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
                    T_end_effector_wrt_base = robot.get_pose()
                    markerArray = MarkerArray()
                    PoseList_tip_rviz = PoseArray()

                    ur_poses = detect_groove_workflow(received_open3d_cloud, T_end_effector_wrt_base.array, detect_feature="asymmetry", show_groove=False)
                    ur_poses = uplift_z(ur_poses)
                    left_poses, right_poses = mutilayer(ur_poses)
                    # print ur_poses
                    trajectory_execution(ur_poses)
                    if mutilayer_exe == True:
                        trajectory_execution(left_poses)
                        trajectory_execution(right_poses)
                    
                    rospy.loginfo("-- Finish display. ...\n")
        else:

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

    rospy.signal_shutdown("Finished shutting down")
