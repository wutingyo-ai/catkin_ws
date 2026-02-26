#!/usr/bin/env python3.8

import math
import rospy
import cv2.dnn
import numpy as np 

from std_msgs.msg import Int32
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point #float型態(x y z)
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32MultiArray

from ultralytics.utils import ASSETS, yaml_load #官網範例程式引入模組
from ultralytics.utils.checks import check_yaml #官網範例程式引入模組

CLASSES = yaml_load(check_yaml("/home/chen/catkin_ws/src/node_control/script/ultralytics/dataset/data.yaml"))["names"] #標注完成後之類別檔案引入
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3)) #依照不同類別改變矩形框顏色


kernel1 = np.ones((3,3),np.uint8)

dist = np.array([0.2309, -1.6024, 0.0023, -0.0010, 3.0824]) #畸變參數
#mtx為相機內參數
mtx = np.array([[2719.4393, 0.0000, 1277.3066, 0.0000],
                       [0.0000, 2721.4259, 939.5802, 0.0000],
                       [0.0000, 0.0000, 1.0000, 0.0000],
                       [0.0000, 0.0000, 0.0000, 1.0000]])

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
        # draw_bounding_box(
        #     original_image,
        #     class_ids[index],
        #     scores[index],
        #     round(box[0] * scale),
        #     round(box[1] * scale),
        #     round((box[0] + box[2]) * scale),
        #     round((box[1] + box[3]) * scale),
        # )
    
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

def main1():

    T_base_e = transform_mtx(3.74, -557.35, 250.00, -179.19, 1.07, 2.3) #法蘭面座標系
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
    Move_ptsx,Move_ptsy = trans_obj(center_pts, T_base_camera, height=173.41)#要移動到的世界座標點位 ；height為s 跟高度有關
    index=0
    print(Move_ptsx[index][0])
    print(Move_ptsy[index][0])
    print(feature_angle[index])
    print(rot_angle[index])
    print(label_sort[index])
        # cv2.imshow("title",img)
    # cv2.waitKey(10)
        # print(rot_angle[0])

main1()