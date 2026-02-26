#!/usr/bin/env python3.8

import argparse #目前用不到
import math
import cv2.dnn
import numpy as np 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point #float型態(x y z)
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32MultiArray
import time
from ultralytics.utils import ASSETS, yaml_load #官網範例程式引入模組
from ultralytics.utils.checks import check_yaml #官網範例程式引入模組


class VisionNode(Node):
    def __init__(self):
        super().__init__('Vision_node')

        # 初始化 Publisher
        self.pub_point_xy = self.create_publisher(Point, 'robot_point_xy', 10)  # 發布膠體 X, Y, Rz 點位
        self.pub_front_reverse = self.create_publisher(String, 'front_reverse', 50)  # 發布正反面字串訊息
        self.pub_Ry_angle = self.create_publisher(Float64, 'Ry_angle', 10)  # 發布開口方向
        self.pub_Place_xy = self.create_publisher(Float32MultiArray, 'Place_xy', 50)  # 發布點位陣列
        self.pub_refill = self.create_publisher(Float32MultiArray, 'refill', 50)  # 發布放置情形
        # 示範如何發布消息
        # self.timer = self.create_timer(0.1, self.publish_messages)  # 每 1 秒發布一次消息
        # 訂閱者
        self.sub_photo = self.create_subscription(Int32, "photo", self.photo_callback, 10)

    def photo_callback(self, msg: Int32):
        """處理收到的 'photo' 訊息"""
        self.get_logger().info(f'Received photo data: {msg.data}')
        #.data後綴是int32的rosmsg文件
        Photo_count = msg.data
        print(Photo_count)
        switch(Photo_count)
    
        #以下發布膠體位置訊息
    def topic_talker(self, point_x, point_y, angle):
        # 建立 Point 訊息
        msg = Point()
        msg.x = point_x
        msg.y = point_y
        msg.z = angle
        # 發布訊息
        self.pub_point_xy.publish(msg)
        self.get_logger().info(f'Published Point: x={msg.x}, y={msg.y}, angle={msg.z}')

        #以下發布正反面訊息
    def topic_front_reverse(self,label):
        msg = String()
        msg.data=label
        
        # print(msg.data,type(msg.data))
        self.pub_front_reverse.publish(msg)

        #以下維膠體開口上下之手臂傾角姿態 +-45度
    def topic_Ry(self,Ry_angle):
        msg=Float64()
        msg.data = float(Ry_angle)
        self.pub_Ry_angle.publish(msg)

    #標定治具點位的傳輸
    def place_array_xy(self,Arr_xy):
        msg = Float32MultiArray() #多維陣列
        msg.layout.dim.append(MultiArrayDimension(label='x', size=20, stride=4))#多維陣列宣告維度
        msg.data = Arr_xy
        self.pub_Place_xy.publish(msg)
    #補料訊息傳輸
    def refill(self,isplaced_list):
        global pub_refill
        
        msg=Float32MultiArray()#
        msg.layout.dim.append(MultiArrayDimension(label='NG',size=10,stride=1))
        msg.data=isplaced_list
        self.pub_refill.publish(msg)

    # def publish_messages(self):
    #     # 構建消息
    #     point_msg = Point(x=1.0, y=2.0, z=0.0)
    #     front_reverse_msg = String(data="正面")
    #     Ry_angle_msg = Float64(data=45.0)
    #     Place_xy_msg = Float32MultiArray(data=[1.0, 2.0, 3.0, 4.0])
    #     refill_msg = Float32MultiArray(data=[0.0, 1.0, 0.0])

    #     # 發布消息
    #     self.pub_point_xy.publish(point_msg)
    #     self.pub_front_reverse.publish(front_reverse_msg)
    #     self.pub_Ry_angle.publish(Ry_angle_msg)
    #     self.pub_Place_xy.publish(Place_xy_msg)
    #     self.pub_refill.publish(refill_msg)

    #     # 打印發布的消息
    #     self.get_logger().info(f'Published Point: {point_msg}')
    #     self.get_logger().info(f'Published Front Reverse: {front_reverse_msg.data}')
    #     self.get_logger().info(f'Published Ry Angle: {Ry_angle_msg.data}')
    #     self.get_logger().info(f'Published Place XY: {Place_xy_msg.data}')
    #     self.get_logger().info(f'Published Refill: {refill_msg.data}')



CLASSES = yaml_load(check_yaml("/home/chen/catkin_ws/src/node_control/script/ultralytics/dataset/data.yaml"))["names"] #標注完成後之類別檔案引入
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3)) #依照不同類別改變矩形框顏色


kernel1 = np.ones((3,3),np.uint8)

dist = np.array([0.2309, -1.6024, 0.0023, -0.0010, 3.0824]) #畸變參數
#mtx為相機內參數
mtx = np.array([[2719.4393, 0.0000, 1277.3066, 0.0000],
                [0.0000, 2721.4259, 939.5802, 0.0000],
                [0.0000, 0.0000, 1.0000, 0.0000],
                [0.0000, 0.0000, 0.0000, 1.0000]])

# pub_point_xy = rospy.Publisher("robot_point_xy" , Point , queue_size = 10 ) #膠體X,Y點位置，還有一RZ礁體旋轉方向 Point=(X,Y,Rz)
# pub_front_reverse = rospy.Publisher("front_reverse" , String , queue_size = 10 ) #正反面字串訊息
# pub_Ry_angle =rospy.Publisher("Ry_angle", Float64, queue_size = 10 ) #開口方向朝上或朝下
# pub_Place_xy = rospy.Publisher("Place_xy", Float32MultiArray, queue_size = 50) #([x1,y1,x2,y2....,x10,y10]) 各點位陣列
# pub_refill=rospy.Publisher('refill',Float32MultiArray,queue_size=50) #[0,1,0...]後檢測放置情形


#以下sub.py Photograph函數發布訊息，判斷接收訊息之條件
def switch(Photo_count):
    if 1 <= Photo_count <= 10:
        main1()
    else:
        main2()
   
    

#4*4轉移矩陣運算方法
def transform_mtx(x, y, z, theta_x, theta_y, theta_z):
    homo = np.array([0, 0, 0])

    T_frame_basex = np.array([[1, 0, 0],
                              [0, math.cos(math.pi*theta_x/180), -math.sin(math.pi*theta_x/180)],    #T_frame_base=np.arry(
                              [0, math.sin(math.pi*theta_x/180), math.cos(math.pi*theta_x/180)]])    # [[r11,r12,r13,tx],
                                                                                                     #  [r21,r22,r23,ty],
    T_frame_basey = np.array([[math.cos(math.pi*theta_y/180), 0, math.sin(math.pi*theta_y/180)],     #  [r31,r32,r33,tz],
                              [0, 1, 0],                                                             #  [ 0,  0,  0,  1]] )
                              [-math.sin(math.pi*theta_y/180), 0, math.cos(math.pi*theta_y/180)]])   #                    

    T_frame_basez = np.array([[math.cos(math.pi*theta_z/180), -math.sin(math.pi*theta_z/180), 0],
                              [math.sin(math.pi*theta_z/180), math.cos(math.pi*theta_z/180), 0],
                              [0, 0, 1]])

    frame_basexy = np.dot(T_frame_basez, T_frame_basey)
    frame_basexyz = np.dot(frame_basexy, T_frame_basex)

    T_frame_base_p = np.array([x, y, z, 1])
    ex_frame_base = np.insert(frame_basexyz, 3, homo, axis = 0) #插入序號3的列向量
    frame_base = np.insert(ex_frame_base, 3, T_frame_base_p, axis = 1) #在序號3插入行向量
    # print(frame_basexyz)
    return frame_base

#手眼校正,請輸入影像點位,以及base至camera的轉移矩陣(外參),還有拍攝高度height,height請參考calculate_matrix.py,得知拍攝高度則可詳閱手冊或棋盤格程式取得內參
def trans_obj(pts, T_base_cam, height): #pts->[u1 u2...un
                                        #      v1 v2...vn ]  n為多少個膠體影像中心，2*10之影像座標矩陣 u是x方向 v是y方向
    move_ptsx = []
    move_ptsy = []
    camera_pts = []
    #k_parm= 4*4相機內參矩陣
    # K_parm = np.array([[2703.4042, 0.0000, 1283.7031, 0.0000],
    #                    [0.0000, 2703.3481, 1032.1431, 0.0000],
    #                    [0.0000, 0.0000, 1.0000, 0.0000],
    #                    [0.0000, 0.0000, 0.0000, 1.0000]])
    K_parm = np.array([[2719.4393, 0.0000, 1277.3066, 0.0000],
                        [0.0000, 2721.4259, 939.5802, 0.0000],
                        [0.0000, 0.0000, 1.0000, 0.0000],
                        [0.0000, 0.0000, 0.0000, 1.0000]])

    pts = np.array(pts)
    K_inv = np.linalg.inv(K_parm) #k_inv= 4*4相機內參反矩陣
    

    for i in range (len(pts[:])):
        
        camerax = pts[i, 0]
        cameray = pts[i, 1]
        camera_pts = height*np.array([[camerax], [cameray], [1], [1/height]]) #(u,v,1,1/s)
        T_cam_o = np.dot(K_inv, camera_pts)
        base_pts = np.dot(T_base_cam, T_cam_o)
        move_ptsx.append(base_pts[0])
        move_ptsy.append(base_pts[1])
    return move_ptsx, move_ptsy


#畫上邊界框顏色
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    """
    Draws bounding boxes on the input image based on the provided arguments.

    Args:
        img (numpy.ndarray): The input image to draw the bounding box on.
        class_id (int): Class ID of the detected object.
        confidence (float): Confidence score of the detected object.
        x (int): X-coordinate of the top-left corner of the bounding box.
        y (int): Y-coordinate of the top-left corner of the bounding box.
        x_plus_w (int): X-coordinate of the bottom-right corner of the bounding box.
        y_plus_h (int): Y-coordinate of the bottom-right corner of the bounding box.
    """
    label = f"{CLASSES[class_id]} ({confidence:.2f})"
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

#onnx_model當作模型權重 input_image影像路徑
def yolov8(onnx_model, input_image): 
    global label_sort,rot_angle,center_pts,feature_angle,open_theta  #label_sort->n個類別依據信心度排列之矩陣:正反反正正...； rot_angle->rz旋轉角 ；center_pts->膠體質心位置 ；feature_angle->開口方向只有+-45度；open_theta->開口向量方向
    """
    Main function to load ONNX model, perform inference, draw bounding boxes, and display the output image.

    Args:
        onnx_model (str): Path to the ONNX model.
        input_image (str): Path to the input image.

    Returns:
        list: List of dictionaries containing detection information such as class_id, class_name, confidence, etc.
    """
    # Load the ONNX model 讀權重
    model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_model)

   # Read the input image 讀影像
    original_image: np.ndarray = cv2.imread(input_image)

    [height, width, _] = original_image.shape

    # Prepare a square image for inference 準備一個影像區域
    length = max((height, width))
    image = np.zeros((length, length, 3), np.uint8)
    image[0:height, 0:width] = original_image
    

    # Calculate scale factor
    scale = length / 640

    # Preprocess the image and prepare blob for model
    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True) #影像前處理
    model.setInput(blob)

    # Perform inference
    outputs = model.forward()
    #print(outputs[0][4][8200])
    # Prepare output array
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]

    boxes = []
    scores = []
    class_ids = []

    label_sort = []
    feature_angle = []
    rot_angle = []
    center_pts = []
    open_theta = []

    # Iterate through output to collect bounding boxes, confidence scores, and class IDs
    for i in range(rows):
        classes_scores = outputs[0][i][4:]
        (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
        if maxScore >= 0.5:
            box = [
                outputs[0][i][0] - (0.5 * outputs[0][i][2]),
                outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                outputs[0][i][2],
                outputs[0][i][3],
            ]
            boxes.append(box)
            scores.append(maxScore)
            class_ids.append(maxClassIndex)
    #print(np.shape(boxes))
    # Apply NMS (Non-maximum suppression)
    result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5) #0.25信心度以下濾掉； iou 0.45取最大聯集 框的最大的；
    # print(result_boxes)
    detections = []
    #print(np.shape(result_boxes))
    # Iterate through NMS results to draw bounding boxes and labels
    for i in range(len(result_boxes)): #膠體的模型辨識資訊
        index = result_boxes[i]
        box = boxes[index]
        label_sort.append(CLASSES[class_ids[index]])
    #嵌入
        cropped_image = original_image[round(box[1] * scale):round((box[1] + box[3]) * scale),round(box[0] * scale):round((box[0] + box[2]) * scale)] #取邊界框內ROI
        gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(3,3), 1)
    
        ret ,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        eroded = cv2.erode(th1,kernel1,iterations=1)
        dilated = cv2.dilate(eroded,kernel1,iterations = 1)
        contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours: #c=各輪廓資訊
            area = cv2.contourArea(c)
            rect = cv2.minAreaRect(c) #輸入影像點位座標資訊，回傳一tuple包含( center (x,y), (width, height), angle of rotation )

            #輪廓面積遮罩 
            if 1800 < area < 3500:
                #print("area:{}".format(area))
                M = cv2.moments(c)
                #單個膠體,影像局部座標
                cpx = int(M['m10']/M['m00']) #非零像素x座標總和/零階矩(面積) = x質心座標
                cpy = int(M['m01']/M['m00']) #非零像素y座標總和/零階矩(面積) = y質心座標
                cen_pt = [cpx, cpy]
                #整張圖像,影像完整座標
                cx = np.round( box[0] * scale + int(M["m10"] / M["m00"]))
                cy = np.round( box[1] * scale +  int(M["m01"]/M["m00"]))
                cen_mass = [cx, cy]
                # print(boxes)
                # print(cen_mass)
                # print(cen_pt)
                bounding_center = np.array(rect[0])#膠體之最小矩形框中心
                    

                if rect[1][0] > rect[1][1]: #width > height
                    angle = np.round(-(float(rect[2])),2) #四捨至小數第2位 
                #print("counterclockwise_rotate")
                else:
                    angle = np.round(90 - float(rect[2]))
                        
                #補償法蘭面的角度
                angle = angle + 90
                #print("angle:{}".format(angle))
                #開口姿態特徵法向量 最小舉行框中心減去膠體質心
                normal = bounding_center - cen_pt
                # print(normal)
                length = (np.sum(np.square(normal)))**0.5
                unit_vec = normal/length #開口姿態特徵法向量單位向量
                theta = math.atan2(unit_vec[1], unit_vec[0])*180/math.pi #開口姿態特徵法向量夾角


                # print(theta)
                if(0 < theta < 180): #開口方向決定取料料姿態
                    Ry = -45
                else:
                    Ry = 45
                    
                    
                center_pts.append(cen_mass)
                rot_angle.append(angle)
                feature_angle.append(Ry)
                open_theta.append(theta)
        # print(feature_angle)

        # cv2.imshow("cropped_image",cropped_image)

        detection = {
            "class_id": class_ids[index],
            "class_name": CLASSES[class_ids[index]],
            "confidence": scores[index],
            "box": box,
            "scale": scale,
        }
        detections.append(detection)
        draw_bounding_box(
            original_image,
            class_ids[index],
            scores[index],
            round(box[0] * scale),
            round(box[1] * scale),
            round((box[0] + box[2]) * scale),
            round((box[1] + box[3]) * scale),
        )
    
    # img_resize=cv2.resize(original_image,(640,640))
    #show=cv2.imwrite("resize.jpg",img_resize)

    # Display the image with bounding boxes
    # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("image",1280,900)    
    # cv2.imshow("image", original_image)
    # cv2.imwrite("/home/chen/save_image/pic.jpg", original_image)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()
    

    return detections


#於轉盤治具上的端子治具標定方法
def calibrate_place():
    
    img = cv2.imread("/home/chen/save_image/pick.jpg")

    raw_crop = (670, -680, 980, -1000)
    adjust_crop = (0, -515, 534, 0)
    #粗分割圖片ROI
    img_crop = img[raw_crop[0]:raw_crop[1], raw_crop[2]:raw_crop[3]]
    

    #細分割圖片ROI(分成左,右找marker)
    img_crop_l = img[raw_crop[0]:raw_crop[1], raw_crop[2] + adjust_crop[0]:raw_crop[3] + adjust_crop[1]]
    img_crop_r = img[raw_crop[0]:raw_crop[1], raw_crop[2] + adjust_crop[2]:raw_crop[3] + adjust_crop[3]]

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
    markers_l = [] #左半邊marker串列
    for c in contours_l:
        Area = cv2.contourArea(c)
        if Area < 70:
            continue
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) + raw_crop[2] + adjust_crop[0]
        cy = int(M["m01"] / M["m00"]) + raw_crop[0]
        markers_l.append((cx, cy))
    markers_l.sort(key = lambda s: s[1])

    markers_r = []#右半邊marker串列
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
        place = (int((m_l[0] + m_r[0]) / 2), int((m_l[1] + m_r[1]) / 2)) #左右兩邊標記之中點距離
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
    #像素總和
    score_tol = 80000
    isplaced_list = []#有無放好之清單 true false true true ...
    for i, p in enumerate(places):
        place_img = img[p[1] - place_crop[0]:p[1] - place_crop[1], p[0] - place_crop[2]:p[0] - place_crop[3]]
        place_img_gray = cv2.cvtColor(place_img, cv2.COLOR_BGR2GRAY)

        score = np.sum(place_img_gray)
        if score > score_tol:
            isplaced_list.append(1.0) # revise
            cv2.circle(img, p, 3, (0, 0, 255), -1)
            cv2.putText(img, str(i), m_l, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            isplaced_list.append(0.0) # revise
            cv2.circle(img, p, 3, (255, 0, 0), -1)
            cv2.putText(img, str(i), m_l, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)

    # print(places)
    # print(isplaced_list)

    # cv2.imshow(str(i), cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2)))
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()

    return places, isplaced_list



def main1():
    global node
    
    T_base_e = transform_mtx(3.74, -557.35, 250.00, -179.19, 1.07, 2.3) #法蘭面座標系
    # T_e_cam = transform_mtx(0.3912, 78.9618, 45.8314, 1.4974, -1.1350, 179.3857) #法蘭至相機轉移矩陣
    T_e_cam = transform_mtx(-0.02, 78.55, 46.3, 1.35, 0.3, -179.2) #法蘭至相機轉移矩陣

    T_base_camera = np.dot(T_base_e, T_e_cam) #外參矩陣

    # img=cv2.imread("/home/chen/save_image/valid/valid6.jpg")
    # output=img
    # alpha=5
    # beta=10
    # cv2.convertScaleAbs(img,output,alpha,beta)
    # cv2.imshow("1",output)
    # cv2.imwrite("/home/chen/save_image/valid/valid6_test.jpg",output)

    yolov8("/home/chen/catkin_ws/src/node_control/script/ultralytics/dataset/runs/1280_1024_1292imgs/1280_1024_1292imgs/weights/best.onnx", "/home/chen/save_image/pick.jpg")

        # print(center_pts)
        # print(feature_angle)
    # 172.94
    # Move_ptsx,Move_ptsy = trans_obj(center_pts, T_base_camera, height=172.94)#要移動到的世界座標點位 ；height為s 跟高度有關
    Move_ptsx,Move_ptsy = trans_obj(center_pts, T_base_camera, height=173.41)#要移動到的世界座標點位 ；height為s 跟高度有關
    # print(Move_ptsx[0][0])
    # print(Move_ptsy[0][0])
    # print(feature_angle[0])
    # print(rot_angle[0])
    # print(label_sort[0])
        # cv2.imshow("title",img)
    # cv2.waitKey(10)
        # print(rot_angle[0])
    time.sleep(1)
    node.topic_front_reverse(label_sort[0])#信心最高的參數發送 發布正反面訊息 
    node.topic_Ry(feature_angle[0])  #信心最高的參數發送 膠體開口上下之手臂傾角姿態 +-45度
    node.topic_talker(Move_ptsx[0][0],Move_ptsy[0][0],rot_angle[0]) #信心最高的參數發送 發布膠體位置訊息




def main2():
        global node
        M_Pts = []
        # T_base_e = transform_mtx(481.65, -520.73, 250.28, 180.00, 0.00, 92.70)#法蘭面座標系
        T_base_e = transform_mtx(462.19, -523.00, 250.27, 180.00, 0.00, 92.12)
        # T_e_cam = transform_mtx(0.39, 78.76, 45.83, 1.49, -1.13, 179.38)#法蘭至相機轉移矩陣
        T_e_cam = transform_mtx(-0.02, 78.55, 46.3, 1.35, 0.3, -179.2) #法蘭至相機轉移矩陣 利用demo askitem求取
        T_base_camera = np.dot(T_base_e, T_e_cam)

        places,isplaced_list = calibrate_place()
        #192.5666
        # Move_ptsx,Move_ptsy = trans_obj(places, T_base_camera, height=173.5381) #要移動到的世界座標點位 ；height為s 跟高度有關 height=176.9366
        Move_ptsx,Move_ptsy = trans_obj(places, T_base_camera, height=189.8915)#要移動到的世界座標點位 ；height為s 跟高度有關 height=176.9366; 2024/10/15 173.5381;
        M_Pts = np.append(Move_ptsx,Move_ptsy)
        # print(type(M_Pts))
        time.sleep(0.5)
        node.place_array_xy(list(M_Pts))
        node.refill(list(isplaced_list))
        
#       topic_Ry(Ry_angle)

# def main3():
#     places,isplaced_list =calibrate_place()
#     refill(isplaced_list)
    
def main():
    rclpy.init()
    global node
    node = VisionNode()
    # rospy.init_node('image_processor', anonymous=True)
    
    # rate = rospy.Rate(0.0001)#要取慢一點
    rate = node.create_rate(0.0001)  # 設置頻率為 10H
    # while not rospy.is_shutdown():

    # rospy.Subscriber("photo", Int32, callback)
        # main1()
        # rate.sleep()

    rclpy.spin(node)
    # try:
    #     while rclpy.ok():
    #         rclpy.spin_once(node)  # 處理一次回調
    #         node.get_logger().info("Loop at 10Hz")
    #         rate.sleep()  # 控制頻率
    # except KeyboardInterrupt:
    #     node.get_logger().info("Shutting down node.")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == "__main__":
    main()