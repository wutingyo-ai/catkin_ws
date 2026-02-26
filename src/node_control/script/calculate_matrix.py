#!/usr/bin/env python3.8
import cv2
import math
import rospy
import numpy as np


def transform_mtx(x, y, z, theta_x, theta_y, theta_z):
    homo = np.array([0, 0, 0])

    T_frame_basex = np.array([[1, 0, 0],
                              [0, math.cos(math.pi*theta_x/180), -math.sin(math.pi*theta_x/180)],
                              [0, math.sin(math.pi*theta_x/180), math.cos(math.pi*theta_x/180)]])

    T_frame_basey = np.array([[math.cos(math.pi*theta_y/180), 0, math.sin(math.pi*theta_y/180)],
                              [0, 1, 0],
                              [-math.sin(math.pi*theta_y/180), 0, math.cos(math.pi*theta_y/180)]])

    T_frame_basez = np.array([[math.cos(math.pi*theta_z/180), -math.sin(math.pi*theta_z/180), 0],
                              [math.sin(math.pi*theta_z/180), math.cos(math.pi*theta_z/180), 0],
                              [0, 0, 1]])

    frame_basexy = np.dot(T_frame_basez, T_frame_basey)
    frame_basexyz = np.dot(frame_basexy, T_frame_basex)

    T_frame_base_p = np.array([x, y, z, 1])
    ex_frame_base = np.insert(frame_basexyz, 3, homo, axis = 0)
    frame_base = np.insert(ex_frame_base, 3, T_frame_base_p, axis = 1)
    # print(frame_basexyz)
    return frame_base
    
# old
# T_base_e = transform_mtx(0.73, -532.83, 250.00, -179.98, -0.02, 0)

T_base_e = transform_mtx(481.11, -520.50, 250.28, 180.00, 0.00, 92.89) #法蘭面座標系
T_e_cam = transform_mtx(-0.02, 78.55, 46.3, 1.35, 0.3, -179.2) #法蘭至相機轉移矩陣
# T_base_e = transform_mtx(482.10, -517.13, 250.00, -178.59, 1.26, 91.50) #法蘭面座標系

# T_e_cam = transform_mtx(0.39, 78.96, 45.83, 1.49, -1.13, 179.38) #文件得知 Handeye Paramter

T_base_camera = np.dot(T_base_e, T_e_cam)

# old
# T_base_o = transform_mtx(27.18, -638.89, 30.18, -179.60, 45.37, 98.91)
# T_base_o = transform_mtx(5.10, 601.11, 29.6, 180.0, -45, 65.34)
# T_base_o = transform_mtx(-10.1828, -663.334, 29, -180.0, -45, 35.06)

# T_base_o = transform_mtx(554.24, -516.61, 28.7, -180.0, 0, 180.00)
T_base_o = transform_mtx(569.84, -517.51, 27.24, -180.00, 0.00, -180.00) #TCP教點

T_cam_base = np.linalg.inv(T_base_camera)

T_cam_o = np.dot(T_cam_base, T_base_o)

# print(T_base_e)
print(T_e_cam)
# print(T_base_camera)
# print(T_base_camera)
print(T_cam_o)