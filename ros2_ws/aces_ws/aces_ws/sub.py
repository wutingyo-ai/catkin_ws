#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageListener(Node):

    def __init__(self,name):
        super().__init__(name)
        self.count = 0  # 計數器 計算夾了第幾次
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Int32, 'photo', 5)  # 建立發布計數器數值消息之發布者
        self.create_subscription(Image, 'techman_image', self.callback, 10)  # 訂閱照片訊息

    def photograph(self, number):  # 發布計數器數值消息函式
        self.pub.publish(Int32(data=number))
        self.get_logger().info(f'Published count: {number}')

    def callback(self, data):
        self.count += 1
        try:
            cv_img = np.asarray(self.bridge.imgmsg_to_cv2(data, '8UC3'))  # CV2須轉格式
            cv2.imwrite("/home/chen/save_image/pick.jpg", cv_img)  # 儲存照片至指定路徑
            self.photograph(self.count)  # 發布計數器數值消息
            self.check_and_reset_count()  # 檢查並重置計數器
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def check_and_reset_count(self):  # 檢查並重置計數器，計數器大於20次則重置計數器為0
        if self.count >= 20:
            self.count = 0

def main(args=None):
    rclpy.init(args=args)
    image_listener = ImageListener("Image_sub")
    rclpy.spin(image_listener)  # 讓節點持續運行直到被手動停止

    # # 當節點被關閉時進行清理
    # image_listener.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
