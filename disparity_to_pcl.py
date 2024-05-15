import open3d
import matplotlib.pyplot as plt
import numpy as np


def DisplayImage(input, colormap):
    plt.imshow(input, cmap=colormap)
    plt.show()

# Set camera intrinsic parmeters.
# intrinsic = open3d.camera.PinholeCameraIntrinsic()
# width (int) Width of the image.
# height (int) Height of the image.
# fx (float) X-axis focal length
# fy (float) Y-axis focal length.
# cx (float) X-axis principle point.
# cy (float) Y-axis principle point.
# intrinsic.set_intrinsics(850, 638, 790, 790, 0,0)
###default for sample data
width = 1280
height = 720
fx = 573.9514770507812
fy = 573.9514770507812
cx = 641.519287109375
cy = 374.7273254394531
baseline = 42.94518572909728

intrinsic = open3d.camera.PinholeCameraIntrinsic(width,height,fx,fy,cx,cy)


# Visualize
vis = open3d.visualization.VisualizerWithKeyCallback()
vis.create_window(width = 600, height = 400)

#depth_raw = open3d.io.read_image('images/disparity/image_0.png')
depth_raw = np.load('images/disparity_kitti12/image_1.npy') * 256
depth_raw = ((baseline * fx) / np.asarray(depth_raw))
depth_raw = np.minimum(depth_raw, 5)
#depth_raw[depth_raw<1] = 5

indices = np.indices(depth_raw.shape)
pcl_np = np.zeros((depth_raw.shape[0]*depth_raw.shape[1],3))
pcl_np[:,0] = ((indices[1]-cx)*depth_raw / fx).reshape(-1)
pcl_np[:,1] = ((indices[0]-cy)*depth_raw / fy).reshape(-1)
pcl_np[:,2] = depth_raw.reshape(-1)
pcl = open3d.geometry.PointCloud()
pcl.points = open3d.utility.Vector3dVector(pcl_np)

depth_raw = open3d.geometry.Image(depth_raw.astype(np.uint16))
DisplayImage(depth_raw, colormap=None)
# pcl = open3d.geometry.PointCloud.create_from_depth_image(depth_raw, intrinsic)
pcl.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
print(open3d.io.write_point_cloud("cloud.pcd", pcl, write_ascii=True, compressed=False, print_progress=True))

vis.add_geometry(pcl)
vis.run()
vis.destroy_window()