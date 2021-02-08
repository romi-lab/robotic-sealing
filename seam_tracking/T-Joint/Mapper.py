from __future__ import division
import open3d as o3d
import time
import copy
from seam_tracking.script import config
import numpy as np


def filter_groove_by_plane(pcd, voxel_size):
    """
    to further filter the groove point by segmented plane
    :param idx_lst: index list of the captured groove point
    :param pcd: point cloud
    :return: the intersection between idx_lst and non-plane points
    """
    pc_number = np.asarray(pcd.points).shape[0]
    entire_set = [i for i in range(pc_number)]
    plane_model, plane_idx = pcd.segment_plane(distance_threshold=voxel_size, ransac_n=50, num_iterations=100)
    print(plane_model)
    pcd_plane = pcd.select_down_sample(plane_idx)
    filter_plane_org_idx = plane_idx
    print("--------start to segment plane:")
    return list(filter_plane_org_idx)

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def extract_workpiece_pcd(pcd):

    plane_idx = filter_groove_by_plane(pcd, 0.001)
    plane_pcd = pcd.select_down_sample(plane_idx)
    plane_pcd_invert = pcd.select_down_sample(plane_idx, invert=True)

    plane_pcd.paint_uniform_color([1, 0, 0])
    plane_pcd_invert.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([plane_pcd, plane_pcd_invert])

    aabb1 = plane_pcd.get_axis_aligned_bounding_box()
    aabb1.color = (1, 0, 0)
    obb1 = plane_pcd.get_oriented_bounding_box()
    obb1.color = (0, 1, 0)
    o3d.visualization.draw_geometries([plane_pcd, aabb1,obb1])
    return plane_pcd

    """Use ICP to classify all samples."""
    labels = None
    for sample in samples:
        results = [icp(sample, template) for template in templates]
        distances = [np.sum(distance) for _, _, distance in results]

        i = int(np.argmin(distances))
        T, s, _ = results[i]  # T (4x4 homogenous transformation)
        label = np.hstack((i, distances[i], np.ravel(T), s, distances)).reshape((1, -1))
        print(' * [Info] Final distance:', distances[i],
              'Label:', template_index_to_name[i])
        labels = label if labels is None else np.vstack((labels, label))
    if labels is None:
        raise UserWarning('No samples found.')
    labels[:, 1] = 1 - (labels[:, 1] / np.max(labels[:, 1]))
    return labels

def CorridnateOfPoint(pcd, idx):
    return np.asarray(pcd.points)[idx]

# ===============================================================================

def draw_registration_result_original_color(source, target, transformation, idx_lst):
    source_temp = copy.deepcopy(source)
    source_temp = source_temp.transform(transformation)
    source_temp.paint_uniform_color([0, 0, 1])

    # create the line set after transformation
    points_labeled = [np.asarray(source_temp.points)[idx_lst[0]], np.asarray(source_temp.points)[idx_lst[1]], np.asarray(source_temp.points)[idx_lst[2]]]
    # print(points_labeled)
    lines = [[0,1], [1,2]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_labeled)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([source_temp, target, line_set])

def MapTemp(pcd, temp, label_idx_lst, log_flag = False):
    """
    :param pcd: captured point cloud
    :param temp: the template point cloud
    :param label_idx_lst: the index of labeled points in template
    :param log_flag: indicator of whether print log
    :return: lines
    """
    # ICP process
    threshold = 0.008
    trans_init = np.asarray([[1, 0., 0, 0],
                             [0, 1., 0, 0],
                             [0, 0., 1, 0],
                             [0, 0., 0, 1]])
    reg_p2p = o3d.registration.registration_icp(temp, pcd, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(max_iteration=6000))
    if log_flag:
        print(reg_p2p)
        print("3.Transformation is:")
        print(reg_p2p.transformation)
    T = reg_p2p.transformation
    idx_lst = label_idx_lst # Points labeled in template
    draw_registration_result_original_color(temp, pcd, T, idx_lst)

if __name__ == "__main__":
    try:
        start = time.time() # record time
        cube_temp = o3d.io.read_point_cloud(config.temp['tcube']['path'])
        cube_label_lst = config.temp['tcube']['label']
        cube_test1 = o3d.io.read_point_cloud("./TempMapping/PCDTemp/StatckedCubeTest9.pcd")
        MapTemp(cube_test1, cube_temp, cube_label_lst)

    finally:
        print("all program ends")