#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import String
i=0

def callback(data):
    global i
    i+=1
    rospy.loginfo("Received message: %s", data.data)
    # 關閉節點
    

def string_subscriber():
    global i
    # 初始化 ROS 節點
    rospy.init_node('string_subscriber', anonymous=True)
    
    # 訂閱 /chatter 主題
    rospy.Subscriber('chatter', String, callback)
    
    # 持續檢查訊息直到收到一次
    while not rospy.is_shutdown():
        rospy.sleep(0.1)  # 確保 CPU 不會過載
        if i==5:
            break
        

if __name__ == '__main__':
    string_subscriber()
    print('Succeess!')
