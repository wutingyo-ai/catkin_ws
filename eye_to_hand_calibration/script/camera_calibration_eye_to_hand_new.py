#!/usr/bin/env python3.8
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob
import os

rs_camera_intrinsic=np.array(
                            [[908.2088623,  0,         644.47814941 ],
                            [  0,       907.76373291 ,361.23849487],
                            [  0,           0,           1        ]])

# 定義轉移矩陣的函數
def R_and_t_to_T(R, t):
    T = np.hstack((R, t.reshape(-1, 1)))  # 將平移向量轉換為列向量
    T = np.vstack((T, [0, 0, 0, 1]))  # 添加最後一行
    return T



######################################################################################棋盤格設置
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
square_size = 0.01

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
objp = objp * square_size

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
path="/home/chen/eye_to_hand_sample/"
# images = sorted(glob.glob(os.path.join(path, '*.png')), key=lambda x: int(os.path.basename(x).split('.')[0]))

select_order=range(0,6)
# 使用 os.path.join() 手動生成檔案名稱
images = [os.path.join(path, f"{i}.png") for i in select_order[:] ]  # 假設有 10 張影像
# images = [os.path.join(path, f"{i}.png") for i in list(range(0, 3)) + list(range(3, 11))]


for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (8,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        # # Draw and display the corners
        cv.drawChessboardCorners(img, (8,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(0)
    print(fname)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], rs_camera_intrinsic,distCoeffs=None ,flags=cv.CALIB_USE_INTRINSIC_GUESS)
######################################################################################

""" #重投影誤差
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
 
print( "total error: {}".format(mean_error/len(objpoints)) ) """


# ####################################################################################### 存放需要的變換矩陣
transformation_matrices_camera=[]
transformation_matrices_camera_Rotation=[]
transformation_matrices_camera_Translation=[]

# Loop through all rotation and translation vectors to create transformation matrices
for rvec, tvec in zip(rvecs, tvecs):
    R = cv.Rodrigues(rvec)[0]  # Convert rotation vector to rotation matrix
    T = R_and_t_to_T(R, tvec)  # Create the transformation matrix


    transformation_matrices_camera_Rotation.append(rvec)
    transformation_matrices_camera_Translation.append(tvec)
    transformation_matrices_camera.append(T)  # Store the transformation matrix



# # Print the transformation matrices
# for idx, T in enumerate(transformation_matrices_camera):
#     print(f'Transformation Matrix for Image {idx}:\n{T}')

""" print(f'intrinsic=\n{mtx}')
print(f'Rotation=\n{rvecs}')
print(f'Translation=\n{tvecs}') 
 """
# print(f'intrinsic=\n{mtx}')
# print(f'Rotation=\n{cv.Rodrigues(rvecs[0])[0]}')
# print(f'Translation=\n{tvecs[0]}') 
######################################################################################

########################################################################################## 讀取手臂姿態訊息
# 讀取 CSV 檔案
data = pd.read_csv('/home/chen/eye_to_hand_sample/flange_pose_values.csv')
# 只選擇第 1 至第 5 行（Python 的索引從 0 開始，因此使用 0:5）在 camera_calibration_eye_to_hand_new.py 程式中，data.iloc[0:6] 會取出 CSV 檔案中的前六行數據，這包括標題行和接下來的五行數據。
data_subset = data.iloc[0:6]
# data_subset = data.iloc[[3,4,5]]

# 存儲轉移矩陣的列表
transformation_matrices_robot_flange = []
transformation_matrices_robot_flange_Rotation = []
transformation_matrices_robot_flange_Translation = []


transformation_matrices_base_to_flange=[]
transformation_matrices_base_to_flange_Rotation=[]
transformation_matrices_base_to_flange_Translation=[]
# 計算轉移矩陣
for index, column in data_subset.iterrows():
    # 提取位置和旋轉數據
    X, Y, Z = column['x'], column['y'], column['z']
    Rx, Ry, Rz = column['rx'], column['ry'], column['rz']
    # print(Rx,Ry,Rz)
    # 計算旋轉矩陣
    print(X,Y,Z,Rx,Ry,Rz)
    rotation = R_scipy.from_euler('xyz', [Rx, Ry, Rz], degrees=True).as_matrix()
    
    # 創建平移向量
    translation = np.array([[X/1000], [Y/1000], [Z/1000]])
    
     
    # 計算 base 到 flange 的轉移矩陣
    rotation_inv = rotation.T  # 旋轉矩陣的逆是其轉置
    translation_inv = -rotation_inv @ translation  # 平移向量的逆


    # 使用自定義函數生成轉移矩陣
    T = R_and_t_to_T(rotation, translation)
    # 生成 base 到 flange 的轉移矩陣
    rotation_inv = np.linalg.inv(T)[:3,:3]
    translation_inv=np.linalg.inv(T)[:3,3]


    T_base_flange = R_and_t_to_T(rotation_inv, translation_inv)
    
    


    # 將轉移矩陣添加到列表中
    transformation_matrices_robot_flange_Rotation.append(rotation)
    transformation_matrices_robot_flange_Translation.append(translation) 
    transformation_matrices_robot_flange.append(T)


   
    transformation_matrices_base_to_flange_Rotation.append(rotation_inv)
    transformation_matrices_base_to_flange_Translation.append(translation_inv)
    transformation_matrices_base_to_flange.append(T_base_flange)#########重要
######################################################################################
# # 檢查轉移矩陣的乘積是否為單位矩陣
# for i in range(len(transformation_matrices_robot_flange)):
#     T_robot_flange = transformation_matrices_robot_flange[i]
#     T_base_flange = transformation_matrices_base_to_flange[i]
    
#     # 計算乘積
#     product = T_robot_flange @ T_base_flange

#     # 檢查是否接近單位矩陣
#     identity_matrix = np.eye(4)  # 4x4 單位矩陣
#     if np.allclose(product, identity_matrix):
#         print(f"Transformation Matrices {i} multiply to identity matrix:\n{product}\n")
#     else:
#         print(f"Transformation Matrices {i} do NOT multiply to identity matrix:\n{product}\n")


# # 輸出轉移矩陣
# for i, T in enumerate(transformation_matrices_robot_flange):
#     print(f"Transformation Matrix {i}:\n{T}\n")



######################################################################################計算相機RGB座標系至手臂基座座標系關係
R_cam2base,t_cam2base=cv.calibrateHandEye(
                                                R_gripper2base = transformation_matrices_base_to_flange_Rotation,
                                                t_gripper2base = transformation_matrices_base_to_flange_Translation,
                                                R_target2cam   = transformation_matrices_camera_Rotation,
                                                t_target2cam   = transformation_matrices_camera_Translation,
                                                method= cv.CALIB_HAND_EYE_TSAI)

print(f'R_cam2base=\nnp.array(\n[[{R_cam2base[0][0]:.3f},{R_cam2base[0][1]:.3f},{R_cam2base[0][2]:.3f}],\n' 
                            f'[{R_cam2base[1][0]:.3f},{R_cam2base[1][1]:.3f},{R_cam2base[1][2]:.3f}],\n'
                            f'[{R_cam2base[2][0]:.3f},{R_cam2base[2][1]:.3f},{R_cam2base[2][2]:.3f}]])\n')

print(f't_cam2base=\nnp.array(\n[[{t_cam2base[0][0]:.3f}],\n[{t_cam2base[1][0]:.3f}],\n[{t_cam2base[2][0]:.3f}]])\n')

T_cam2base=R_and_t_to_T(R_cam2base,t_cam2base)
print(f'T_cam2base=\nnp.array(\n[[{T_cam2base[0][0]:.3f},{T_cam2base[0][1]:.3f},{T_cam2base[0][2]:.3f},{T_cam2base[0][3]:.3f}],\n' 
                            f'[{T_cam2base[1][0]:.3f},{T_cam2base[1][1]:.3f},{T_cam2base[1][2]:.3f},{T_cam2base[1][3]:.3f}],\n'
                            f'[{T_cam2base[2][0]:.3f},{T_cam2base[2][1]:.3f},{T_cam2base[2][2]:.3f},{T_cam2base[2][3]:.3f}],\n'
                            f'[{T_cam2base[3][0]:.3f},{T_cam2base[3][1]:.3f},{T_cam2base[3][2]:.3f},{T_cam2base[3][3]:.3f}]])\n')

# print(f'R_cam2base=\n{R_cam2base},\n t_cam2base=\n{t_cam2base}')

######################################################################################



# 定義從 screw 到 camera 的平移和旋轉
translation_screw_to_camera = np.array([0.032, 0.013, -0.010])


# 計算從 screw 到 camera 的旋轉矩陣
quaternion_screw_to_camera = np.array([0.499, -0.500, 0.500, 0.501])
rotation_screw_to_camera = R_scipy.from_quat(quaternion_screw_to_camera).as_matrix() 

# 建立從 screw 到 camera 的轉移矩陣 (4x4)
T_screw_to_camera = np.eye(4)
T_screw_to_camera[:3, :3] = rotation_screw_to_camera
T_screw_to_camera[:3, 3] = translation_screw_to_camera

""" # 定義從 camera 到 base 的旋轉矩陣和平移
R_cam2base = np.array(
[[0.932, 0.126, 0.339],
[-0.357, 0.164, 0.920],
[0.060, -0.978, 0.198]])

t_cam2base = np.array([[-0.360], [-0.875], [0.132]]) """

# 建立從 camera 到 base 的轉移矩陣 (4x4)
T_camera_to_base = np.eye(4)
T_camera_to_base[:3, :3] = R_cam2base
T_camera_to_base[:3, 3] = t_cam2base.flatten()

# 計算從 screw 到 base 的轉移矩陣
T_screw_to_base = T_camera_to_base @ T_screw_to_camera
R_screw_to_base = T_screw_to_base[:3, :3]
t_screw_to_base = T_screw_to_base[:3, 3]

# 輸出結果
print("從camera_rgb_frame 到 base 的轉移矩陣 ")
print(T_camera_to_base)

print("從 screw 到 base 的轉移矩陣:")
print(T_screw_to_base)

# 將旋轉矩陣轉換為歐拉角 (XYZ)
rotation = R_scipy.from_matrix(R_screw_to_base)
euler_angles = rotation.as_euler('xyz', degrees=True)

# 輸出位移和旋轉角度
print(f"平移向量 (XYZ): {t_screw_to_base}")
print(f"旋轉角度radians (RX, RY, RZ): {np.radians(euler_angles)}")

# 在這裡你可以發布 TF 轉換訊息
# 例如，使用 rospy 發布 TF 訊息的範例
# import rospy
# import tf
# broadcaster = tf.TransformBroadcaster()
# broadcaster.sendTransform(
#     t_screw_to_base,
#     rotation.as_quat(),
#     rospy.Time.now(),
#     "base",
#     "screw"
# )

# 繪製座標系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 基座座標系的原點
base_origin = np.array([0, 0, 0])

# 繪製 base 座標系 (紅色)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          1, 0, 0, color='r', label='Base X', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 1, 0, color='g', label='Base Y', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 0, 1, color='b', label='Base Z', length=0.5)

# 繪製 cam 螺絲底座 座標系
cam_bottom_screw_origin = t_screw_to_base
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 0], R_screw_to_base[1, 0], R_screw_to_base[2, 0], 
          color='r', label='cam_bottom_screw_origin_X', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 1], R_screw_to_base[1, 1], R_screw_to_base[2, 1], 
          color='g', label='cam_bottom_screw_origin_Y', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 2], R_screw_to_base[1, 2], R_screw_to_base[2, 2], 
          color='b', label='cam_bottom_screw_origin_Z', length=0.5)

# 繪製 camera 到 base 的座標系
camera_rgb_frame_origin= t_cam2base
ax.quiver(camera_rgb_frame_origin[0], camera_rgb_frame_origin[1], camera_rgb_frame_origin[2], 
          R_cam2base[0, 0], R_cam2base[1, 0], R_cam2base[2, 0], 
          color='c', label='Camera X', length=0.5)
ax.quiver(camera_rgb_frame_origin[0], camera_rgb_frame_origin[1], camera_rgb_frame_origin[2], 
          R_cam2base[0, 1], R_cam2base[1, 1], R_cam2base[2, 1], 
          color='m', label='Camera Y', length=0.5)
ax.quiver(camera_rgb_frame_origin[0], camera_rgb_frame_origin[1], camera_rgb_frame_origin[2], 
          R_cam2base[0, 2], R_cam2base[1, 2], R_cam2base[2, 2], 
          color='y', label='Camera Z', length=0.5)

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



""" 平移向量 (XYZ): [-0.331928 -0.893492  0.119226]
旋轉角度 (RX, RY, RZ): [ -3.63431022 -11.30655766  69.75749912] """


""" 2025/03/28
R_cam2base=
np.array(
[[-0.020,0.905,-0.425],
[0.995,0.059,0.078],
[0.096,-0.422,-0.902]])

t_cam2base=
np.array(
[[0.940],
[-0.139],
[0.623]])

T_cam2base=
np.array(
[[-0.020,0.905,-0.425,0.940],
[0.995,0.059,0.078,-0.139],
[0.096,-0.422,-0.902,0.623],
[0.000,0.000,0.000,1.000]]) """