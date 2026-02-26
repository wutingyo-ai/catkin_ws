#!/usr/bin/env python3
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob
import os

rs_camera_intrinsic=np.array(
                            [[909.78143311,  0,         638.0090332 ],
                            [  0,       909.79705811 ,370.18139648],
                            [  0,           0,           1        ]])

# 定義轉移矩陣的函數
def R_and_t_to_T(R, t):
    T = np.hstack((R, t.reshape(-1, 1)))  # 將平移向量轉換為列向量
    T = np.vstack((T, [0, 0, 0, 1]))  # 添加最後一行
    return T
def R_and_t_to_T_minus(R, t):
    # 將 Z 軸取負
    t[2] = -t[2]
    T = np.hstack((R, t.reshape(-1, 1)))  # 將平移向量轉換為列向量
    T = np.vstack((T, [0, 0, 0, 1]))  # 添加最後一行
    return T


#####################################################################################
""" 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
square_size = 0.01  # 每個棋盤格的邊長 (米)

# 準備物體點
objp = np.zeros((8*6, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
objp = objp * square_size

# Arrays to store object points and image points from all the images.
objpoints = []  # 3D point in real world space
imgpoints = []  # 2D points in image plane

path="/home/chen/eye_to_hand_origin/"
images = sorted(glob.glob(os.path.join(path, '*_Color.png')), key=lambda x: int(os.path.basename(x).split('_')[0]))

# 讀取所有圖像並偵測棋盤格角點
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # 找到棋盤格角點
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)

# 相機標定
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# print('calibrateCamera')
# print('rvecs=', rvecs[1], '\n', 'tvecs=', tvecs[1], '\n')


transformation_matrices_camera=[]
transformation_matrices_camera_Rotation=[]
transformation_matrices_camera_Translation=[]

# 繪製坐標系
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # 找到棋盤格角點
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 計算相機姿態
        retval, rvec, tvec = cv.solvePnP(objp, corners2, mtx, dist)

        # 繪製坐標系
        axis_length = 0.1  # 坐標系的長度為 0.1 米
        cv.drawFrameAxes(img, mtx, dist, rvec, tvec, axis_length, 3)  # 繪製坐標系

        R = cv.Rodrigues(rvec)[0]  # Convert rotation vector to rotation matrix
        T = R_and_t_to_T(R, tvec)  # Create the transformation matrix


        transformation_matrices_camera_Rotation.append(R)
        transformation_matrices_camera_Translation.append(tvec)
        transformation_matrices_camera.append(T)  # Store the transformation matrix   


        cv.imshow('img', img)
        cv.waitKey(0)

cv.destroyAllWindows()

 """



######################################################################################
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

path="/home/chen/eye_to_hand_origin/"
# images = sorted(glob.glob(os.path.join(path, '*_Color.png')), key=lambda x: int(os.path.basename(x).split('_')[0]))
images = [os.path.join(path, f"{i}_Color.png") for i in range(1,30)]  

# # 使用 os.path.join() 手動生成檔案名稱
# images = [os.path.join(path, f"{i}_Color.png") for i in range(1, 11)]  # 假設有 10 張影像

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




# #重投影誤差
# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#     mean_error += error
 
# print( "total error: {}".format(mean_error/len(objpoints)) )
# ######################################################################################



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


""" for rvec, tvec in zip(rvecs, tvecs):
    R = cv.Rodrigues(rvec)[0]  # Convert rotation vector to rotation matrix
    tvec[2] = -tvec[2]  # 反轉 Z 軸方向
    T = R_and_t_to_T(R, tvec)  # Create the transformation matrix

    transformation_matrices_camera_Rotation.append(R)
    transformation_matrices_camera_Translation.append(tvec)
    transformation_matrices_camera.append(T)  # Store the transformation matrix
 """
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

#########################################################################################


# 讀取 CSV 檔案
data = pd.read_csv('/home/chen/eye_to_hand_origin/eye_to_hand_flange.csv')
data_subset = data.iloc[:30]
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
    X, Y, Z = column['X'], column['Y'], column['Z']
    Rx, Ry, Rz = column['Rx'], column['Ry'], column['Rz']
    # print(Rx,Ry,Rz)
    # 計算旋轉矩陣
    # print(X,Y,Z,Rx,Ry,Rz)
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




R_cam2base,t_cam2base=cv.calibrateHandEye(
                                                R_gripper2base = transformation_matrices_base_to_flange_Rotation,
                                                t_gripper2base = transformation_matrices_base_to_flange_Translation,
                                                R_target2cam   = transformation_matrices_camera_Rotation,
                                                t_target2cam   = transformation_matrices_camera_Translation,
                                                method= cv.CALIB_HAND_EYE_TSAI)

print(f'R_cam2base=\n{R_cam2base},\n t_cam2base=\n{t_cam2base}')







""" 
def R_and_t_to_T(R, t):
    T = np.hstack((R, t))
    T = np.vstack((T, [0, 0, 0, 1]))
    return T

def T_to_R_and_t(T):
    Rt = T[:3]
    R = Rt[:, :3]
    t = Rt[:, 3].reshape((-1, 1))
    return R, t

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

images = glob.glob("/home/chen/eye_to_hand_origin/*.png")

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
        # Draw and display the corners
        # cv.drawChessboardCorners(img, (8,6), corners2, ret)
        # cv.imshow('img', img)
        # cv.waitKey(500)
    print(fname)


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

transformation_matrixes=[]

# Loop through all rotation and translation vectors to create transformation matrices
for rvec, tvec in zip(rvecs, tvecs):
    R = cv.Rodrigues(rvec)[0]  # Convert rotation vector to rotation matrix
    T = R_and_t_to_T(R, tvec)  # Create the transformation matrix
    transformation_matrixes.append(T)  # Store the transformation matrix

# Print the transformation matrices
for idx, T in enumerate(transformation_matrixes):
    print(f'Transformation Matrix for Image {idx}:\n{T}')


# print(f'intrinsic=\n{mtx}')
# print(f'Rotation=\n{cv.Rodrigues(rvecs[0])[0]}')
# print(f'Translation=\n{tvecs[0]}') """

""" import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd


# 定義平移和旋轉參數
x, y, z = 10, 20, 50
rx, ry, rz = -100, -50, 10  # 角度

# 將角度轉換為弧度
rx_rad = np.radians(rx)
ry_rad = np.radians(ry)
rz_rad = np.radians(rz)

# 計算旋轉矩陣
rotation_matrix = R.from_euler('xyz', [rx_rad, ry_rad, rz_rad]).as_matrix()


# 創建平移向量
translation_vector = np.array([[x], [y], [z]])

# 創建 4x4 的轉移矩陣
T = np.eye(4)  # 創建單位矩陣
T[:3, :3] = rotation_matrix  # 填入旋轉矩陣
T[:3, 3] = translation_vector.flatten()  # 填入平移向量

print("4x4 轉移矩陣：")
print(T) """


""" import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd


# 定義平移和旋轉參數
x, y, z = 10, 20, 50
rx, ry, rz = -100, -50, 10  # 角度

# 將角度轉換為弧度
rx_rad = np.radians(rx)
ry_rad = np.radians(ry)
rz_rad = np.radians(rz)

# 計算旋轉矩陣
rotation_matrix = R.from_euler('xyz', [rx_rad, ry_rad, rz_rad]).as_matrix()


# 創建平移向量
translation_vector = np.array([[x], [y], [z]])

# 創建 4x4 的轉移矩陣
T = np.eye(4)  # 創建單位矩陣
T[:3, :3] = rotation_matrix  # 填入旋轉矩陣
T[:3, 3] = translation_vector.flatten()  # 填入平移向量

print("4x4 轉移矩陣：")
print(T)  """

