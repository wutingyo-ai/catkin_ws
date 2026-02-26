#!/usr/bin/env python3.8

import cv2
import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError #ros用於傳輸圖片之函式


count = 0 #計數器 計算夾了第幾次
bridge = CvBridge()
pub = rospy.Publisher("photo", Int32, queue_size = 5 ) #建立發布計數器數值消息之發布者

def Photograph(number):  #發布計數器數值消息函式
    global pub
    pub.publish(number)
    print(number)

def callback(data):
    global count, bridge, cv_img
    count += 1
    cv_img = np.asarray(bridge.imgmsg_to_cv2(data, '8UC3')) #CV2須轉格式z
    cv2.imwrite(f"/home/chen/save_image/pick.jpg", cv_img) #儲存散料區膠體照片至指定路徑
    #回調順便發布訊息
    Photograph(count) #發布計數器數值消息

    #此為迴圈條件重置
    check_and_reset_count() #此為迴圈條件重置，計數器大於20次則重置計數器為0

    # "/home/chen/save_image/pick.jpg"
    #"/home/chen/save_image/place.jpg"
    #專案路徑
    # "/home/chen/save_image/training_sample/sample"+str(count)+".jpg"
    # cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("frame",1280,900)
    # cv2.imshow("frame" ,cv_img)
    # cv2.waitKey(0)

def check_and_reset_count(): #此為迴圈條件重置，計數器大於20次則重置計數器為0
    global count
    if count == 20:
        count = 0


def webcamImagePub(): #主函示運行，不斷訂閱照片訊息
    global bridge, pub
    rospy.init_node('image_listener', anonymous=True)
    rate = rospy.Rate(10)  # Adjust the rate as needed
    img_sub = rospy.Subscriber('techman_image', Image, callback)
    rospy.spin()


       
if __name__ == '__main__': #主函示運行，不斷訂閱照片訊息
    webcamImagePub()
    
    