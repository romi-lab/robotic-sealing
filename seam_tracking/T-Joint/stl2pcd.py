from __future__ import division
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import time
import sys
import trimesh
import copy


mesh1 = trimesh.load_mesh("./Model_STL/TwoPieces.STL")
mesh2 = trimesh.load_mesh("./Model_STL/TwoPieces2.STL")

mesh1.export("./Model_STL/TwoPieces.ply")
mesh2.export("./Model_STL/TwoPieces2.ply")


mesh1_o3d = o3d.io.read_triangle_mesh("./Model_STL/TwoPieces.ply")
mesh1_o3d.compute_vertex_normals()
# o3d.visualization.draw_geometries([mesh1_o3d])
pcd_STL = mesh1_o3d.sample_points_uniformly(number_of_points=40000)
o3d.visualization.draw_geometries([pcd_STL])
o3d.io.write_point_cloud("TwoPieces.pcd", pcd_STL)




