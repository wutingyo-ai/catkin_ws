#!/usr/bin/env python3.8
import cv2
import math
import rospy
import numpy as np
import math

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

def trans_obj(pts,T_base_cam,height):
    K_parm = np.array([[2719.4393, 0.0000, 1277.3066, 0.0000],
                    [0.0000, 2721.4259, 939.5802, 0.0000],
                    [0.0000, 0.0000, 1.0000, 0.0000],
                    [0.0000, 0.0000, 0.0000, 1.0000]])
    
    K_inv = np.linalg.inv(K_parm)
    
    pts = np.array(pts)

    camera_pts = []
    move_ptsx = []
    move_ptsy = []
    for i in range (len(pts[:])):
        
        camerax = pts[i, 0]
        cameray = pts[i, 1]
        camera_pts = height*np.array([[camerax], [cameray], [1], [1/height]])
        T_cam_o = np.dot(K_inv, camera_pts)
        base_pts = np.dot(T_base_cam, T_cam_o)
        move_ptsx.append(base_pts[0])
        move_ptsy.append(base_pts[1])
    return move_ptsx, move_ptsy

def calibrate_place():
    
    img = cv2.imread("/home/chen/save_image/pick.jpg")
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow("image",1280,900)    
    cv2.imshow("image", img)
    cv2.waitKey(0)

    raw_crop = (670, -980, 980, -1000)
    adjust_crop = (0, -515, 534, 0)
    #粗分割圖片ROI
    img_crop = img[raw_crop[0]:raw_crop[1], raw_crop[2]:raw_crop[3]]
    
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow("image",1280,900)    
    cv2.imshow("image", img_crop)
    cv2.waitKey(0)

    #細分割圖片ROI(分成左,右找marker)
    img_crop_l = img[raw_crop[0]:raw_crop[1], raw_crop[2] + adjust_crop[0]:raw_crop[3] + adjust_crop[1]]
    img_crop_r = img[raw_crop[0]:raw_crop[1], raw_crop[2] + adjust_crop[2]:raw_crop[3] + adjust_crop[3]]

    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow("image",1280,900)    
    cv2.imshow("image", img_crop_l)
    cv2.waitKey(0)

    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow("image",1280,900)    
    cv2.imshow("image", img_crop_r)
    cv2.waitKey(0)

    #轉灰階
    img_crop_gray_l = cv2.cvtColor(img_crop_l, cv2.COLOR_BGR2GRAY)
    img_crop_gray_r = cv2.cvtColor(img_crop_r, cv2.COLOR_BGR2GRAY)

    #高斯模糊
    kernel_size = 5
    blur_gray_l = cv2.GaussianBlur(img_crop_gray_l, (kernel_size, kernel_size), 0)
    blur_gray_r = cv2.GaussianBlur(img_crop_gray_r, (kernel_size, kernel_size), 0)

    #二值化+找輪廓
    lower_thesh = 130
    upper_thesh = 255
    _, thresh_l = cv2.threshold(blur_gray_l, lower_thesh, upper_thesh, cv2.THRESH_BINARY)
    _, thresh_r = cv2.threshold(blur_gray_r, lower_thesh, upper_thesh, cv2.THRESH_BINARY)
    contours_l, _ = cv2.findContours(thresh_l, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_r, _ = cv2.findContours(thresh_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #找輪廓質點,分類marker順序
    markers_l = []
    for c in contours_l:
        Area = cv2.contourArea(c)
        if Area < 70:
            continue
        print(Area)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) + raw_crop[2] + adjust_crop[0]
        cy = int(M["m01"] / M["m00"]) + raw_crop[0]
        markers_l.append((cx, cy))
    markers_l.sort(key = lambda s: s[1])

    markers_r = []
    for c in contours_r:
        Area = cv2.contourArea(c)
        if Area < 70:
            continue
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) + raw_crop[2] + adjust_crop[2]
        cy = int(M["m01"] / M["m00"]) + raw_crop[0]
        markers_r.append((cx, cy))
    markers_r.sort(key = lambda s: s[1])

    #找膠體放置點
    places = []
    angs = []
    for i, (m_l, m_r) in enumerate(zip(markers_l, markers_r)):
        place = (int((m_l[0] + m_r[0]) / 2), int((m_l[1] + m_r[1]) / 2))
        places.append(place)
        #角度,很重要
        x = m_r[0] - m_l[0]
        y = m_r[1] - m_l[1]
        ang = math.degrees(math.atan(y / x))
        angs.append(ang)
        
        cv2.putText(img, str(ang), place, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, str(i), m_l, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, str(i), m_r, cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(img, m_l, 2, (255, 0, 0), -1)
        cv2.circle(img, m_r, 2, (0, 255, 0), -1)
        cv2.line(img, m_l, m_r, (0, 0, 255), 1)
        
    #確認膠體放置點是否有膠體
    place_crop = (5, -5, 25, -25)
    score_tol = 80000
    isplaced_list = []
    for i, p in enumerate(places):
        place_img = img[p[1] - place_crop[0]:p[1] - place_crop[1], p[0] - place_crop[2]:p[0] - place_crop[3]]
        place_img_gray = cv2.cvtColor(place_img, cv2.COLOR_BGR2GRAY)

        score = np.sum(place_img_gray)
        if score > score_tol:
            isplaced_list.append(True)
            cv2.circle(img, p, 3, (0, 0, 255), -1)
            cv2.putText(img, str(i), m_l, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            isplaced_list.append(False)
            cv2.circle(img, p, 3, (255, 0, 0), -1)
            cv2.putText(img, str(i), m_l, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)

    #print(places)
    #print(isplaced_list)
    # print(angs)
    cv2.imshow(str(i), cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2)))
    cv2.waitKey(0)

    return places, isplaced_list

# T_base_e = transform_mtx(481.12, -520.50, 250.28, 180.00, 0.00, 92.89)
T_base_e = transform_mtx(481.65, -520.73, 250.28, 180.00, 0.00, 92.70)
T_e_cam = transform_mtx(-0.02, 78.55, 46.3, 1.35, 0.3, -179.2) #法蘭至相機轉移矩陣 利用demo askitem求取


T_base_camera = np.dot(T_base_e, T_e_cam)

places,isplaced_list=calibrate_place()

# Move_ptsx,Move_ptsy = trans_obj(places, T_base_camera,height=174.936)
# Move_ptsx,Move_ptsy = trans_obj(places, T_base_camera, height=173.5381)#要移動到的世界座標點位 ；height為s 跟高度有關 1.769366
Move_ptsx,Move_ptsy = trans_obj(places, T_base_camera, height=189.8915)#要移動到的世界座標點位 ；height為s 跟高度有關 height=176.9366; 2024/10/15 173.5381;
print(Move_ptsx)
print("")
print(Move_ptsy)
#offset 0.6092


#[array([261.15730933]), array([259.1809272]), array([257.14579554]), array([255.16820759]), array([253.19122255]), array([251.2136346]), array([249.23664956]), array([247.2015179]), array([245.22453286]), array([243.24694491])]
#[array([-469.34540221]), array([-469.42429096]), array([-469.4456435]), array([-469.40823969]), array([-469.42898216]), array([-469.39157835]), array([-469.41232082]), array([-469.43367337]), array([-469.45441584]), array([-469.41701203])]


#[array([261.1603239]), array([259.12458933]), array([257.14760429]), array([255.17061925]), array([253.1930313]), array([251.15729672]), array([249.17970877]), array([247.20272373]), array([245.22573869]), array([243.24875365])]
#[array([-469.6361336]), array([-469.59933986]), array([-469.62008233]), array([-469.6408248]), array([-469.60342099]), array([-469.56662726]), array([-469.52922345]), array([-469.54996592]), array([-469.57070839]), array([-469.59145086])]