# %% Reference: https://darkpgmr.tistory.com/190
''' Take KITTI dataset's lidar and camera data and calibrate using the provided calibration matrix '''
import numpy as np
import matplotlib.pyplot as plt
from utils import read_bin_velodyne, plot_3d_shape_np, read_kitti_calib_txt

# %%
pcd = read_bin_velodyne('./sample_data/lidar_000003.bin')
img = plt.imread('./sample_data/image_000003.png')
calib = read_kitti_calib_txt( './sample_data/calib_000003.txt')

# %%
plt.imshow(img); plt.show()

# %%
print(pcd.shape)
print(pcd)
print(calib)

# %%
plot_3d_shape_np(pcd)

# %%
P2 = calib['P2']
R0_rect = calib['R0_rect']
Tr_velo_to_cam = calib['Tr_velo_to_cam']

# %% s(x, y, 1) = P2 * R0_rect * Tr_velo_to_cam * (X, Y, Z, 1)
n_data = pcd.shape[0]
pcd_3d = np.hstack((pcd[:, :3], np.ones((n_data, 1))))

xy1 = np.matmul(P2, np.vstack((np.matmul(R0_rect, np.matmul(Tr_velo_to_cam, pcd_3d.T)), np.ones((1, n_data))))).T
s = xy1[:, 2]
x = xy1[:, 0] / s
y = xy1[:, 1] / s

plt.scatter(x, y, s=1, c=pcd[:, 3], cmap='gray')
x_lim = img.shape[1]
y_lim = img.shape[0]
plt.axis('equal')
plt.xlim(0, x_lim)
plt.ylim(y_lim, 0)
plt.show()

plt.imshow(img); plt.show()

# %% Filter out points that are behind the camera (s < 0)
n_data = pcd.shape[0]
pcd_3d = np.hstack((pcd[:, :3], np.ones((n_data, 1))))

xy1 = np.matmul(P2, np.vstack((np.matmul(R0_rect, np.matmul(Tr_velo_to_cam, pcd_3d.T)), np.ones((1, n_data))))).T
s = xy1[:, 2]
x = xy1[:, 0] / s
y = xy1[:, 1] / s
k = s > 0

plt.scatter(x[k], y[k], s=1, c=pcd[k, 3], cmap='gray')
x_lim = img.shape[1]
y_lim = img.shape[0]
plt.axis('equal')
plt.xlim(0, x_lim)
plt.ylim(y_lim, 0)
plt.show()

plt.imshow(img); plt.show()

# %%
