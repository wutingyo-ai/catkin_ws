#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定義從 screw 到 camera 的平移和旋轉
translation_screw_to_camera = np.array([0.032, 0.013, -0.010])
quaternion_screw_to_camera = np.array([0.499, -0.500, 0.500, 0.501])

# 計算從 screw 到 camera 的旋轉矩陣
rotation_screw_to_camera = R.from_quat(quaternion_screw_to_camera).as_matrix()

# 建立從 screw 到 camera 的轉移矩陣 (4x4)
T_screw_to_camera = np.eye(4)
T_screw_to_camera[:3, :3] = rotation_screw_to_camera
T_screw_to_camera[:3, 3] = translation_screw_to_camera

# 定義從 camera 到 base 的旋轉矩陣和平移
R_cam2base = np.array(
[[0.932,0.126,0.339],
[-0.357,0.164,0.920],
[0.060,-0.978,0.198]])

t_cam2base = np.array(
[[-0.360],
[-0.875],
[0.132]])

# 建立從 camera 到 base 的轉移矩陣 (4x4)
T_camera_to_base = np.eye(4)
T_camera_to_base[:3, :3] = R_cam2base
T_camera_to_base[:3, 3] = t_cam2base.flatten()

# 計算從 screw 到 base 的轉移矩陣
T_screw_to_base = T_camera_to_base @ T_screw_to_camera
R_screw_to_base = T_screw_to_base[:3, :3]
t_screw_to_base = T_screw_to_base[:3, 3]
# 輸出結果
print("從 screw 到 base 的轉移矩陣:")
print(T_screw_to_base)

R_cam2base = R_screw_to_base
                       

t_cam2base = t_screw_to_base



# 基座座標系的原點
base_origin = np.array([0, 0, 0])

# 繪製座標系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 繪製 base 座標系 (紅色)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          1, 0, 0, color='r', label='Base X', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 1, 0, color='g', label='Base Y', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 0, 1, color='b', label='Base Z', length=0.5)

# 繪製 cam 座標系
cam_origin = t_cam2base
ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
          R_cam2base[0, 0], R_cam2base[1, 0], R_cam2base[2, 0], 
          color='r', label='Cam X', length=0.5)
ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
          R_cam2base[0, 1], R_cam2base[1, 1], R_cam2base[2, 1], 
          color='g', label='Cam Y', length=0.5)
ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
          R_cam2base[0, 2], R_cam2base[1, 2], R_cam2base[2, 2], 
          color='b', label='Cam Z', length=0.5)

# 設定圖形屬性
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Base and Camera Coordinate Systems')
ax.legend()
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

plt.show()


""" 從 screw 到 base 的轉移矩陣:
[[ 0.33925319 -0.93174681 -0.12718549 -0.331928  ]
 [ 0.92032545  0.35732545 -0.16144535 -0.893492  ]
 [ 0.19604373 -0.06195627  0.97827209  0.119226  ]
 [ 0.          0.          0.          1.        ]]"""