# %%
import struct
import numpy as np
import plotly.express as px

# %%
def read_bin_velodyne(path):
    ''' Read velodyne binary file and return a numpy array of shape (n,3) '''
    pc_list=[]
    with open(path,'rb') as f:
        content = f.read()
        pc_iter = struct.iter_unpack('ffff',content)
        for idx, point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2], point[3]]) # 4th point for intensity
    return np.asarray(pc_list,dtype=np.float32)

# %%
def read_kitti_calib_txt(calib_txt_path):
    ''' Read kitti calibration txt file and return a numpy array of shape (3,4) '''
    keys = ['P0','P1','P2','P3','R0_rect','Tr_velo_to_cam','Tr_imu_to_velo']

    with open(calib_txt_path,'r') as f:
        lines = f.readlines()
    lines = [line.strip().split(' ')[1:] for line in lines]

    for idx, line in enumerate(lines[:6]):
        if keys[idx]=='R0_rect':
            lines[idx] = np.asarray(line,dtype=np.float32).reshape(3,3)
        else:
            lines[idx] = np.asarray(line,dtype=np.float32).reshape(3,4)
    return dict(zip(keys,lines))

# %%
def plot_3d_shape_np(array):
    print('Number of data points: ', array.shape[0])
    x = array[:, 0]
    y = array[:, 1]
    z = array[:, 2]
    fig = px.scatter_3d(x=x, y=y, z=z, opacity=0.3)
    fig.update_traces(marker_size=3)
    fig.show()
