# Created by Jeffery at 19:26 2/7/21 using PyCharm
from __future__ import division
import numpy as np
import open3d as o3d

import sys
from ctypes import * # convert float to uint32
import os
from datetime import datetime
import time
import urx
from matplotlib import pyplot as plt
import copy
from mpl_toolkits.mplot3d import Axes3D
import scipy.spatial as spatial
from sklearn.neighbors import KDTree
from scipy.spatial.transform import Rotation as R
from scipy import interpolate
import copy
import math
import vg


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

def points_in_cylinder(pt1, pt2, r, query_points):
    """
    to check if a query point is in the cylinder defined as
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


def thin_line(points, point_cloud_thickness=0.5, iterations=1, sample_points=0):
    total_start_time = time.time()
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
        nn_time += time.time() - start_time

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
        point_moved = linepts[0] + np.dot(ap, ab) / np.dot(ab, ab) * ab
        prj_time += time.time() - start_time

        new_points.append(list(point_moved))
    # print("--- %s seconds to thin points ---" % (time.time() - total_start_time))
    # print(f"Finding nearest neighbors for calculating regression lines: {nn_time}")
    # print(f"Calculating regression lines: {rl_time}")
    # print(f"Projecting original points on  regression lines: {prj_time}\n")
    return np.array(new_points), regression_lines


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
    (tck, u), fp, ier, msg = interpolate.splprep([x, y, z], s=float("inf"), full_output=1)
    # (tck, u), fp, ier, msg = interpolate.splprep([x_pos, y_pos, z_pos], s=float("inf"),full_output=1)
    # print tck
    # print u
    # line_fit = Line.best_fit(points)
    # points = line_fit.project_point(points)
    # Generate 5x points from approximated B-spline for drawing curve later
    u_fine = np.linspace(0, 1, x.size * 2)

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

def detect_groove_workflow(pcd):
    global max_dis, total_time, voxel_size, delete_percentage
    original_pcd = pcd

    voxel_size = 0.001
    # pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # pcd_points = np.asarray(pcd.points)
    # pcd.clear()
    # pcd_points = pcd_points[pcd_points[:,2]<max_dis]
    # pcd.points = o3d.utility.Vector3dVector(pcd_points)
    pcd.remove_none_finite_points()

    pc_number = np.asarray(pcd.points).shape[0]
    print("Total number of pc {}".format(pc_number))

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
         radius=0.01, max_nn=30))
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location(camera_location=[0., 0., 0.])
    pcd.paint_uniform_color([0.8, 0.8, 0.8])

    # Start to fetch the points set with human interaction
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    print(vis.get_picked_points())
    pick_idx_lst = vis.get_picked_points()
    print("The indexes of the selected points are: \n {} ".format(pick_idx_lst))
    # p1 - p2, cylinder
    # p2 - p3, cylinder
    # produce points
    # get the index list of ROIs in the original down sampled PCD
    r = voxel_size * 18
    start_p = np.asarray(pcd.points)[pick_idx_lst[0]]
    corner_p_true = np.asarray(pcd.points)[pick_idx_lst[1]]
    end_p = np.asarray(pcd.points)[pick_idx_lst[2]]
    corner_p = corner_p_true + (corner_p_true - start_p) * 0.11
    ROI_points_lst1 = points_in_cylinder(start_p, corner_p, r, np.asarray(pcd.points))
    corner_p = corner_p_true + (corner_p_true - end_p) * 0.11
    ROI_points_lst2 = points_in_cylinder(end_p, corner_p, r, np.asarray(pcd.points))
    ROI_points_lst = np.concatenate((ROI_points_lst1, ROI_points_lst2), axis = 0)
    # ROI_points_lst = ROI_points_lst1 # for testing
    ROI_pcd = o3d.geometry.PointCloud()
    ROI_pcd.points = o3d.utility.Vector3dVector(ROI_points_lst)
    ROI_pcd.paint_uniform_color([1, 0.5, .5])
    pcd.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([pcd, ROI_pcd])


    points = [start_p, corner_p_true, end_p]
    # print(points.shape)
    lines = [[0, 1], [1, 2]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector3dVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([ROI_pcd, line_set])

    return ROI_pcd



    trajectory = generate_trajectory(pcd, ROI_pcd)
    normal = find_normal(trajectory, pcd)
    points = np.asarray(trajectory.points)
    point_size_line = points.shape[0]
    start_point = points[0]
    end_point = points[point_size_line-1]






    # feautre_value_list = find_feature_value(detect_feature, pcd, voxel_size)
    # normalised_feautre_value_list = normalise_feautre(feautre_value_list)


if __name__ == "__main__":
    # Basic settings
    is_first_pose = True
    is_sec_pose = False
    voxel_size = 0.005
    neighbor = 5*voxel_size
    delete_percentage = 0.95
    received_ros_cloud = None
    max_dis = 0.7

    received_open3d_cloud = o3d.io.read_point_cloud("./pc0002.pcd")
    pcd = detect_groove_workflow(received_open3d_cloud)
    # o3d.visualization.draw_geometries([pcd])
    # o3d.io.write_point_cloud("Test.pcd", pcd)


    pass