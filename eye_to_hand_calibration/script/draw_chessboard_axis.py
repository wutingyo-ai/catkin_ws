#!/usr/bin/env python3
import pandas as pd
import numpy as np
import cv2 as cv
import glob

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

images = glob.glob("/home/chen/eye_to_hand_origin/*.png")

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
print('calibrateCamera')
print('rvecs=', rvecs[1], '\n', 'tvecs=', tvecs[1], '\n')

# 繪製坐標系
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # 找到棋盤格角點
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        print('Pnp')
        
        # 計算相機姿態
        retval, rvec, tvec = cv.solvePnP(objp, corners2, mtx, dist)

        # 繪製坐標系
        axis_length = 0.1  # 坐標系的長度為 0.1 米
        cv.drawFrameAxes(img, mtx, dist, rvec, tvec, axis_length, 3)  # 繪製坐標系

        # 顯示結果
        cv.imshow('img', img)
        cv.waitKey(0)

cv.destroyAllWindows()





""" # !/usr/bin/env python3
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob




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

images = glob.glob("/home/chen/eye_to_hand_origin/*.png")

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
print('calibrateCamera')
print('rvecs=',rvecs[1],'\n','tvecs=',tvecs[1],'\n')

i=0
# 繪製坐標系
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # 找到棋盤格角點
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        print('Pnp')
        # 計算相機姿態
        retval, rvec, tvec = cv.solvePnP(objp, corners2, mtx, dist)

        
        # print('rvec=',np.round(rvec-rvecs[i],4),'tvec=',np.round(tvec-tvecs[i],4),)
        # i+=1

        # 繪製坐標系
        axis = np.float32([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]).reshape(-1, 3)  # 坐標系的長度為 0.1 米
        imgpts, _ = cv.projectPoints(axis, rvec, tvec, mtx, dist)

        # 繪製坐標系
        origin = tuple(corners2[0][0].astype(int))
        img = cv.line(img, origin, tuple(imgpts[0].ravel().astype(int)), (0, 0, 255), 5)  # X 軸 (r)
        img = cv.line(img, origin, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 5)  # Y 軸 (g)
        img = cv.line(img, origin, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 5)  # Z 軸 (b)

        # 顯示結果
        cv.imshow('img', img)
        cv.waitKey(0)

cv.destroyAllWindows() """