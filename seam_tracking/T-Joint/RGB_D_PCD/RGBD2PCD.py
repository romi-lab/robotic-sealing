# Created by Jeffery at 17:16 1/22/21 using PyCharm
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

print("Workpiece Dataset Test")
color_raw = o3d.io.read_image("./color_00002.png")
depth_raw = o3d.io.read_image("./depth_00002.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw)
# print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('Workpiece Grayscale Image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Workpiece Depth Image')
plt.imshow(rgbd_image.depth)
plt.show()

print(np.asarray(rgbd_image))

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
int_M = np.asarray(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
# o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
print(int_M)
# print("the intrinsic matrix is \n {} ".format(np.asarray(o3d.camera.PinholeCameraIntrinsic.intrinsic_matrix)))
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# o3d.visualization.draw_geometries([pcd])
print(640*480)
print(len(np.asarray(pcd.points)))

idx = [233 * 151, 351 * 131, 299 * 183]
idx =  [ 351 * 131 + i for i in range(100)]
print(idx)
points_2d= pcd.select_down_sample(idx)
points_2d.paint_uniform_color([0, 1, 0])
# o3d.visualization.draw_geometries([pcd, points_2d])

# pcd_output = o3d.io.read_point_cloud("./pc0002.pcd")
# print(len(np.asarray(pcd_output.points)))
