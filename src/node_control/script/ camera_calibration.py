#!/usr/bin/env python3
# import cv2
# import numpy as np
# import glob

# image = cv2.imread("/home/chen/save_image/chess.jpg")

# objp = np.zeros((7*5, 3), np.float32)
# objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
# objp = 10*objp
# obj_points = []
# img_points = []
# images = glob.glob("/home/chen/save_image/camera_calibration/*.jpg")

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img , cv2.COLOR_BGR2GRAY)
#     size = gray.shape[::-1]
#     ret, corners = cv2.findChessboardCorners(gray, (7, 5), None)
#     if ret:
#         obj_points.append(objp)
#         corners2 = cv2.cornerSubPix(gray, corners, (10, 10), (-1, -1),
#                                     (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001))

#         if [corners2]:
#             img_points.append(corners2)
#         else:
#             img_points.append(corners)
#         cv2.drawChessboardCorners(img, (7, 5), corners, ret)
#         cv2.waitKey(1)
# _, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)

# Camera_intrinsic = {"mtx" : mtx, "dist" : dist,}
# print("內參")
# print(mtx)
# print("畸變")
# print(dist)

# dst = cv2.undistort(image, mtx, dist)

# cv2.namedWindow("pic",0)
# cv2.resizeWindow("pic", 2800, 1400)
# cv2.imshow("pic", dst)
# cv2.waitKey(0)



import numpy as np
import cv2 as cv
import glob

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

images = glob.glob("/home/chen/save_image/camera_calibration/*.jpg")

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
    print(fname)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
print(dist)