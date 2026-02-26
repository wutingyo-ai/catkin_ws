#!/usr/bin/env python3

import rospy
from gpd_ros.msg import GraspConfigList

def grasp_callback(msg):
    rospy.loginfo("收到 %d 個抓取姿態", len(msg.grasps))
    for i, grasp in enumerate(msg.grasps):
        rospy.loginfo("抓取姿態 #%d: 位置 = (%.3f, %.3f, %.3f)", 
                    i, 
                    grasp.position.x, 
                    grasp.position.y, 
                    grasp.position.z)

def main():
    rospy.init_node("grasp_subscriber", anonymous=True)
    
    # 訂閱 /detect_grasps/clustered_grasps 話題
    # 如果您的話題名稱不同，請相應修改
    rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, grasp_callback)
    
    rospy.loginfo("抓取姿態訂閱者節點已啟動，等待消息...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
