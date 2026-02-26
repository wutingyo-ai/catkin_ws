#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('user_input_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        user_input = input("請輸入要發布的訊息: ")
        rospy.loginfo(user_input)
        pub.publish(user_input)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass