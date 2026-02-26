#!/usr/bin/env python3

import cv2
import math
import rospy
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import Point 

img = cv2.imread("/home/chen/save_image/pick.jpg")


#特徵法向量
bounding_center = []
cen_mass = []
#hsv mask的上下限根據環境光源調整,參考find_object_hsv.py,多放幾個膠體,確保全部都能夠辨識
lower_white = np.array([0, 0, 55])
upper_white = np.array([180, 60, 255])

kernel1 = np.ones((3,3),np.uint8)

# 畸變校正,請參考張正友校正法,找出內參與畸變參數,程式參考camera_correction
mtx = np.array([[2.7136e+03, 0.0000e+00, 1.3035e+03],
                [0.0000e+00, 2.7160e+03, 1.0127e+03],
                [0.0000e+00, 0.0000e+00, 1.0000e+00]])
dist = np.array([0.03997643, 0.03689492, -0.00096063, 0.00292095, -0.85829774])
img = cv2.undistort(img, mtx, dist)

# net = cv2.dnn.readNet('/home/chen/backup/yolov4-custom_final.weights', '/home/chen/backup/cfg/yolov4-custom.cfg')

# with open("/home/chen/backup/cfg/obj.names", "r") as f:
#     classes = f.read().splitlines()


# def topic_front_reverse(label):
    
#     pub = rospy.Publisher("front_reverse" , String , queue_size = 10 )
#     #rospy.init_node("talker1", anonymous=True)
#     label_info = label
#     pub = pub.publish(label_info)
#     rate = rospy.Rate(10)
#     rate.sleep()


def topic_talker(grab_pointx, grab_pointy, angle):
    pub = rospy.Publisher("robot_point_xy" , Point , queue_size = 10 )
    rospy.init_node("talker2132131231", anonymous=True)
    x = grab_pointx
    y = grab_pointy  
    z = angle 
    msg = Point(x,y,z)
    pub = pub.publish(msg)

    rate = rospy.Rate(10)
    rate.sleep()




def mask_process(image):

    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_white = cv2.inRange(hsv_img, lower_white, upper_white)
    trans_ori = cv2.bitwise_and(img ,img, mask = mask_white)
    #交集原圖並使用遮罩,變回bgr

    return trans_ori


def take_process(image):
    feature_angle = []
    rot_angle = []
    center_pts = []


    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3), 1)
    
    ret ,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    eroded = cv2.erode(th1,kernel1,iterations=1)
    dilated = cv2.dilate(eroded,kernel1,iterations = 1)
    contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #輪廓左至右排序 詳查sorted函數
    contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

    for c in contours:
        area = cv2.contourArea(c)
        #輪廓面積遮罩 
        if area < 2000 or area > 4000:
            continue
        
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        M =cv2.moments(c)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        bounding_center = np.array(rect[0])
        cen_mass = [cx, cy]
        print(cen_mass)
        if rect[1][0] > rect[1][1]: #w > h
            angle = np.round(-(float(rect[2])),2)
           #print("counterclockwise_rotate")
        else:
            angle = np.round(90 - float(rect[2]))
        #補償法蘭面的角度
        angle = angle + 61.16
        
    #開口姿態特徵法向量
        normal = bounding_center - cen_mass
        # print(normal)
        length = (np.sum(np.square(normal)))**0.5
        unit_vec = normal/length
        theta = math.atan2(unit_vec[1], unit_vec[0])*180/math.pi


        img_contour = cv2.drawContours(image, [box], -1, (0, 0, 255), 3)
        
        center_pts.append(cen_mass)
        rot_angle.append(angle)
        feature_angle.append(theta)
        # print(rect[2])    
        # print(area)
        # print("rot_angle:{}".format(rot_angle))
        # print(center_pts)

    return img_contour, feature_angle, center_pts ,rot_angle


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

def trans_obj(pts,T_base_cam):
    
    # K_parm = np.array([[2.78499011e+03, 0.00000000e+00, 1.28812490e+03, 0.00000000e+00],
    #                    [0.00000000e+00, 2.78549656e+03, 1.04362381e+03, 0.00000000e+00],
    #                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00],
    #                    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    K_parm = np.array([[2.7136e+03, 0.0000e+00, 1.3035e+03, 0.0000e+00],
                       [0.0000e+00, 2.7160e+03, 1.0127e+03, 0.0000e+00],
                       [0.0000e+00, 0.0000e+00, 1.0000e+00, 0.0000e+00],
                       [0.0000e+00, 0.0000e+00, 0.0000e+00, 1.0000e+00]])

    K_inv = np.linalg.inv(K_parm)
    
    pts = np.array(pts)
    # print(pts)
    #從TM_flow獲取到法蘭面的轉換矩陣,TM_ROS獲取法蘭面到相機的轉換矩陣(2D)
    camera_pts = []
    move_ptsx = []
    move_ptsy = []
    #將center_pts帶入運算,並產生新的點位數組
    for i in range (len(pts[:])):
        
        camerax = pts[i, 0]
        cameray = pts[i, 1]
        # 依照當下高度的投影透視
        camera_pts = 170*np.array([[camerax], [cameray], [1], [1/170]])
        T_cam_o = np.dot(K_inv, camera_pts)
        # print(T_cam_o)
        base_pts = np.dot(T_base_cam, T_cam_o)
        move_ptsx.append(base_pts[0])
        move_ptsy.append(base_pts[1])

    # print("move_ptsx:{}".format(move_ptsx))
    # print("move_ptsx[0]:{}".format(move_ptsx[0][0]))
    # print("move_ptsy:{}".format(move_ptsy))
    # print("move_ptsy[0]:{}".format(move_ptsy[0][0]))
    return move_ptsx, move_ptsy



wellimg = mask_process(img)
pro = take_process(wellimg)

T_base_e = transform_mtx(0.73, -532.83, 250.00, -179.98, -0.02, 0.00)
T_e_cam = transform_mtx(0.39, 78.96, 45.83, 1.49, -1.13, 179.38)

T_base_camera = np.dot(T_base_e, T_e_cam)

# print(pro[1])
Move_ptsx,Move_ptsy = trans_obj(pro[2], T_base_camera)
# print("pose:{}".format(Move_ptsx[0][0]))
# data_pub = topic_talker(pose[0], pose[1], pro[3])

print(Move_ptsx[0][0])
print(Move_ptsy[0][0])
topic_talker(Move_ptsx[0][0], Move_ptsy[0][0], pro[3][0])

cv2.namedWindow("pic",0)
cv2.resizeWindow("pic", 900, 600)
cv2.imshow("pic", pro[0])
cv2.waitKey(0)