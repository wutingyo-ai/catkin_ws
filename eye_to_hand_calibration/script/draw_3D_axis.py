#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# # 定義旋轉矩陣和位移向量
# R_cam2base = np.array([[ 0.98892985, -0.09406213, -0.11476094],
#                        [ 0.0535957 ,  0.94762488, -0.31485645],
#                        [ 0.13836639,  0.30522025,  0.94217585]])

                       

# t_cam2base = np.array([-0.08155043, -0.40121386, 0.21272556])

R_cam2base = np.array(
[[0.932,0.126,0.339],
[-0.357,0.164,0.920],
[0.060,-0.978,0.198]])

                       

t_cam2base = np.array(
[[-0.360],
[-0.875],
[0.132]])



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