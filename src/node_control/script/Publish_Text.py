#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import String



def string_publisher():
    # 初始化 ROS 節點
    rospy.init_node('string_publisher', anonymous=True)
    
    # 創建一個 Publisher，發布到 /chatter 主題
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # 設定發布頻率
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # 要發布的字串
        hello_str = "Hello, ROS! %s" % rospy.get_time()
        
        # 發布字串
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        # 等待下一次發布
        rate.sleep()

if __name__ == '__main__':
    try:
        string_publisher()
    except rospy.ROSInterruptException:
        pass
