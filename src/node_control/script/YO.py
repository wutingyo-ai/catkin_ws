#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import math
import rospy
import numpy as np
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point 
net = cv2.dnn.readNet('/home/chen/backup_1102/yolov4-custom_final.weights', '/home/chen/backup_1102/cfg_1102/yolov4-custom.cfg')

classes = []
with open("/home/chen/backup_1102/cfg_1102/obj.names", "r") as f:
    classes = f.read().splitlines()

kernel1 = np.ones((3,3),np.uint8)

# old
# dist = np.array([0.03997643, 0.03689492, -0.00096063, 0.00292095, -0.85829774])

dist = np.array([0.01737176, -0.11378668, -0.0019655, -0.00259464, 0.52787566])


mtx = np.array([[2.7052e+03, 0.0000e+00, 1.2790e+03],
                [0.0000e+00, 2.7079e+03, 1.0170e+03],
                [0.0000e+00, 0.0000e+00, 1.0000e+00]])

def topic_talker(point_x,point_y,angle):
    pub = rospy.Publisher("robot_point_xy" , Point , queue_size = 1000 )
    #rospy.init_node("talker2132131231", anonymous=True)
    x = point_x
    y = point_y 
    z = angle 
    msg = Point(x,y,z)
    pub = pub.publish(msg)

    rate = rospy.Rate(1000)
    rate.sleep()

def topic_front_reverse(label):
    
    pub = rospy.Publisher("front_reverse" , String , queue_size = 10 )
    #rospy.init_node("talker1", anonymous=True)
    label_info = label
    pub = pub.publish(label_info)
    rate = rospy.Rate(10)
    rate.sleep()

def topic_Ry(Ry_angle):
    pub =rospy.Publisher("Ry_angle", Float64, queue_size = 10 )
    Ry = Ry_angle
    msg = Float64(Ry)
    pub = pub.publish(msg)
    rate = rospy.Rate(10)
    rate.sleep()

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
    #old
    # K_parm = np.array([[2.7849e+03, 0.0000e+00, 1.2881e+03, 0.0000e+00],
    #                    [0.0000e+00, 2.7854e+03, 1.0436e+03, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 1.0000e+00, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 0.0000e+00, 1.0000e+00]])
    
    K_parm = np.array([[2.7052e+03, 0.0000e+00, 1.2790e+03, 0.0000e+00],
                       [0.0000e+00, 2.7079e+03, 1.0170e+03, 0.0000e+00],
                       [0.0000e+00, 0.0000e+00, 1.0000e+00, 0.0000e+00],
                       [0.0000e+00, 0.0000e+00, 0.0000e+00, 1.0000e+00]])
    
    #接近答案
    # K_parm = np.array([[2.7136e+03, 0.0000e+00, 1.2881e+03, 0.0000e+00],
    #                    [0.0000e+00, 2.7160e+03, 1.0276e+03, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 1.0000e+00, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 0.0000e+00, 1.0000e+00]])

    #接近答plus
    # K_parm = np.array([[2.7136e+03, 0.0000e+00, 1.2881e+03, 0.0000e+00],
    #                    [0.0000e+00, 2.7160e+03, 1.0176e+03, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 1.0000e+00, 0.0000e+00],
    #                    [0.0000e+00, 0.0000e+00, 0.0000e+00, 1.0000e+00]])
    

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
        #173.97
        #172.94
        #174.66
        camera_pts = 174.74*np.array([[camerax], [cameray], [1], [1/174.74]])
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

def yolov4(img):
    global label_sort,rot_angle,center_pts,feature_angle,open_theta

    label_sort = []
    height, width, _ = img.shape


    blob = cv2.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
    net.setInput(blob)
    output_layers_names = net.getUnconnectedOutLayersNames()
    layerOutputs = net.forward(output_layers_names)
    boxes = []
    confidences = []
    class_ids = []

    feature_angle = []
    rot_angle = []
    center_pts = []
    open_theta = []

    for output in layerOutputs:
        for detection in output:
            scores = detection[5:]
            
            class_id = np.argmax(scores)#0 front 1 rev 
                           
            confidence = scores[class_id]
            if confidence > 0.7:
                #print(class_id) 
                #print(detection)
                center_x = int(detection[0]*width)
                center_y = int(detection[1]*height)
                w = int(detection[2]*width)
                h = int(detection[3]*height)
                x = int(center_x - w/2)
                y = int(center_y - h/2)
                boxes.append([x, y, w, h])
                # print(boxes)
                confidences.append((float(confidence)))
                
                class_ids.append(class_id)
                #print(class_ids)
                #if class_ids[0] == 1:   #class_ids[] = [1, ,1 , 1 ]之類的 最多三個
        
                

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)
    if len(indexes)>0:
        
        for i in indexes.flatten():
            x,y = (boxes[i][0], boxes[i][1]) 
            w,h = (boxes[i][2], boxes[i][3])
            label = str(classes[class_ids[i]])
            

            confidence = str(round(confidences[i],2))
            #yolo程式訓練出來的結果會有點不准,所以才需要加上輪廓之類的影像處理來加以辨識#
            cropped_image = img[boxes[i][1]:boxes[i][1]+boxes[i][3],boxes[i][0]:boxes[i][0]+boxes[i][2]]
            
            # print(boxes) 
            
            gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray,(3,3), 1)
    
            ret ,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            eroded = cv2.erode(th1,kernel1,iterations=1)
            dilated = cv2.dilate(eroded,kernel1,iterations = 1)
            contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            label_sort.append(label)

            #輪廓左至右排序 詳查sorted函數
            # contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

            for c in contours:
                area = cv2.contourArea(c)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int_(box)
                #print("area:{}".format(area))topic_talker
                #輪廓面積遮罩 
                if 2000 < area < 3700:
                    #print("area:{}".format(area))
                    M = cv2.moments(c)
                    #單個膠體,影像局部座標
                    cpx = int(M['m10']/M['m00'])
                    cpy = int(M['m01']/M['m00'])
                    cen_pt = [cpx, cpy]
                    #整張圖像,影像世界座標
                    cx = np.round( boxes[i][0] + int(M["m10"] / M["m00"]))
                    cy = np.round( boxes[i][1] +  int(M["m01"]/M["m00"]))
                    cen_mass = [cx, cy]
                    # print(boxes)
                    # print(cen_mass)
                    # print(cen_pt)
                    bounding_center = np.array(rect[0])
                    

                    if rect[1][0] > rect[1][1]: #w > h
                        angle = np.round(-(float(rect[2])),2)
                    #print("counterclockwise_rotate")
                    else:
                        angle = np.round(90 - float(rect[2]))
                        
                    #補償法蘭面的角度
                    angle = angle + 90
                    #print("angle:{}".format(angle))
                    #開口姿態特徵法向量
                    normal = bounding_center - cen_pt
                    # print(normal)
                    length = (np.sum(np.square(normal)))**0.5
                    unit_vec = normal/length
                    theta = math.atan2(unit_vec[1], unit_vec[0])*180/math.pi


                    # print(theta)
                    if(0 < theta < 180):
                        Ry = -45
                    else:
                        Ry = 45
                    
                    # img_contour = cv2.drawContours(image, [box], -1, (0, 0, 255), 3)
                    
                    center_pts.append(cen_mass)
                    rot_angle.append(angle)
                    # print(theta)
                    feature_angle.append(Ry)
                    open_theta.append(theta)
                    # print("rot_angle:{}".format(rot_angle))
                    
                    cv2.rectangle(img, (x,y), (x+w, y+h), (100,0,255), 4) #左上,右下
                    cv2.putText(img, label + ":" + confidence, (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                    cv2.circle(cropped_image, (cpx,cpy), 2, (0,0,255) , -1)
            cv2.imshow("cropped_image",cropped_image)
    return img

def main():
    
    while(1):

        img = cv2.imread("/home/chen/save_image/pick.jpg")

        
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0.2, (w,h))
        x, y, w, h = roi
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        dst = dst[y:y+h, x:x+w]

        cv2.namedWindow('title', cv2.WINDOW_NORMAL)
        cv2.resizeWindow("title",1280,900)

        T_base_e = transform_mtx(3.74, -538.35, 250.00, -179.19, 1.07, -0.02)
        T_e_cam = transform_mtx(0.39, 78.76, 45.83, 1.49, -1.13, 179.38)


        T_base_camera = np.dot(T_base_e, T_e_cam)
        image = yolov4(dst)

        # print(center_pts)
        # print(feature_angle)

        Move_ptsx,Move_ptsy = trans_obj(center_pts, T_base_camera)
        # print(Move_ptsx[0][0])
        # print(Move_ptsy[0][0])
        cv2.imshow("title",img)
        cv2.waitKey(10)
        # print(rot_angle[0])

        topic_front_reverse(label_sort[0])
        topic_Ry(feature_angle[0])
        
        topic_talker(Move_ptsx[0][0],Move_ptsy[0][0],rot_angle[0])
        
        # if (0 > open_theta[0] > -90):
        #     topic_talker(Move_ptsx[0][0]+3.5*math.cos(open_theta[0]*math.pi/180),Move_ptsy[0][0]+3.5*math.sin(open_theta[0]*math.pi/180),rot_angle[0])
        # elif (90 < open_theta[0] < 180):
        #     topic_talker(Move_ptsx[0][0]+3.5*math.cos(open_theta[0]*math.pi/180),Move_ptsy[0][0]+3.5*math.sin(open_theta[0]*math.pi/180),rot_angle[0])   
        # else:
        #     topic_talker(Move_ptsx[0][0],Move_ptsy[0][0],rot_angle[0])

        # elif (90 < open_theta[0] < 180):
        #     topic_talker(Move_ptsx[0][0]-0.04,Move_ptsy[0][0]-0.59,rot_angle[0])
            

        # elif (0 > open_theta[0] > -90):
        #     topic_talker(Move_ptsx[0][0]+4.5*math.cos(open_theta[0]*math.pi/180),Move_ptsy[0][0]+4.5*math.sin(open_theta[0]*math.pi/180),rot_angle[0])
            




        # print(open_theta[0])

        if cv2.waitKey(1) & 0xff == ord("q"):
                break
    
if __name__== '__main__':
    rospy.init_node("YOLOv4_and_grab_point", anonymous=True)
    main()
    cv2.destroyAllWindows()

    