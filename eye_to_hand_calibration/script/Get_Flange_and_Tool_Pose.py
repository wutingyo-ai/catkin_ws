#!/usr/bin/env python3

import rospy
from tm_msgs.srv import AskItem, AskItemRequest
import re
import numpy as np
from std_msgs.msg import String
import os

def extract_pose_values(coord_string):
    # 使用正則表達式提取花括號內的內容
    match = re.search(r'\{(.*?)\}', coord_string)
    if match:
        # 將提取的內容按逗號分割
        values_str = match.group(1).split(',')
        # 將字符串轉換為浮點數
        values = [float(v.strip()) for v in values_str]
        return values
    else:
        return None

# 回调函数，用于处理接收到的字符串消息
def string_callback(msg):
    global flange_pose_values_list, tool_pose_values_list, start_subscribing
    SAVE_PATH = "/home/chen/eye_to_hand_sample"
    if msg.data == "ok":
        rospy.loginfo("Received 'ok', starting to subscribe to pose messages.")
        start_subscribing = True
    elif msg.data == "finish":
        rospy.loginfo("Received 'finish', saving pose values to CSV files.")
        start_subscribing = False
        # 保存为CSV文件
        if flange_pose_values_list is not None:
            np.savetxt(os.path.join(SAVE_PATH, 'flange_pose_values.csv'), flange_pose_values_list, delimiter=',', header='x,y,z,rx,ry,rz', comments='')
            rospy.loginfo("Saved flange_pose_values_list to flange_pose_values.csv")
        if tool_pose_values_list is not None:
            np.savetxt(os.path.join(SAVE_PATH,'tool_pose_values.csv'), tool_pose_values_list, delimiter=',', header='x,y,z,rx,ry,rz', comments='')
            rospy.loginfo("Saved tool_pose_values_list to tool_pose_values.csv")

def main():
    global flange_pose_values_list, tool_pose_values_list, start_subscribing
    
    rospy.init_node('demo_ask_item')

    # 初始化变量
    flange_pose_values_list = None
    tool_pose_values_list = None
    start_subscribing = False

    # 创建字符串消息的订阅者
    rospy.Subscriber("user_input", String, string_callback)

    # 创建服务客户端
    client = rospy.ServiceProxy('tm_driver/ask_item', AskItem)
    request = AskItemRequest()
    request.id = "demo"
    request.wait_time = 0.5

    rate = rospy.Rate(1)  # 10Hz
    
    while not rospy.is_shutdown():
        if start_subscribing:
            start_subscribing=False
            # 订阅 Coord_Robot_Flange
            request.item = "Coord_Robot_Flange"
            try:
                response = client(request)
                if response.ok:
                    rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(request.item, response.value))
                    coord_string = response.value
                    pose_values = extract_pose_values(coord_string)
                    if pose_values:
                        pose_values = np.asarray(pose_values)
                        if flange_pose_values_list is None:
                            flange_pose_values_list = pose_values
                        else:
                            flange_pose_values_list = np.vstack((flange_pose_values_list, pose_values))
                    else:
                        rospy.logwarn("Failed to extract flange pose values")
                else:
                    rospy.logwarn("AskItem to robot, but response not yet ok")
                # print(flange_pose_values_list)
            except rospy.ServiceException as e:
                rospy.logerr("Error AskItem to robot: {}".format(e))

            # 订阅 Coord_Robot_Tool
            request.item = 'Coord_Robot_Tool'
            try:
                response = client(request)
                if response.ok:
                    rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(request.item, response.value))
                    coord_string = response.value
                    pose_values = extract_pose_values(coord_string)
                    if pose_values:
                        pose_values = np.asarray(pose_values)
                        if tool_pose_values_list is None:
                            tool_pose_values_list = pose_values
                        else:
                            tool_pose_values_list = np.vstack((tool_pose_values_list, pose_values))
                    else:
                        rospy.logwarn("Failed to extract tool pose values")
                else:
                    rospy.logwarn("AskItem to robot, but response not yet ok")
            except rospy.ServiceException as e:
                rospy.logerr("Error AskItem to robot: {}".format(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass





###########################################################
""" import rospy
from tm_msgs.srv import AskItem, AskItemRequest
import re
import numpy as np

def extract_pose_values(coord_string):
    # 使用正則表達式提取花括號內的內容
    match = re.search(r'\{(.*?)\}', coord_string)
    if match:
        # 將提取的內容按逗號分割
        values_str = match.group(1).split(',')
        # 將字符串轉換為浮點數
        values = [float(v.strip()) for v in values_str]
        return values
    else:
        return None


def main():
    rospy.init_node('demo_ask_item')
    
    # 創建服務客戶端
    client = rospy.ServiceProxy('tm_driver/ask_item', AskItem)
    
    # 創建服務請求
    request = AskItemRequest()
    request.id = "demo"
    request.item = "Coord_Robot_Flange"
    # request.item = "g_complete_signal"
    request.wait_time = 0.5
    #  使用示例
    flange_pose_values_list=None
    tool_pose_values_list=None

    try:
        # 調用服務並等待結果
        response = client(request)
        
        if response.ok:
            # rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(response.id, response.value))
            rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(request.item , response.value))
            coord_string = response.value
            pose_values = extract_pose_values(coord_string)

            if pose_values:
                x, y, z, rx, ry, rz = pose_values
                pose_values=np.asarray(pose_values)
                
                if flange_pose_values_list==None:
                    flange_pose_values_list=pose_values
                else:
                    flange_pose_values_list=np.vstack((flange_pose_values_list,pose_values))

                            
           


                print(flange_pose_values_list)
            else:
                print("Failed to extract pose values")
            
        else:
            rospy.logwarn("AskItem to robot, but response not yet ok")
        print('\n')
        
############################################################################
        request.item='Coord_Robot_Tool'
        response = client(request)
        if response.ok:
            rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(response.id, response.value))
            coord_string = response.value
            pose_values = extract_pose_values(coord_string)

            if pose_values:
                x, y, z, rx, ry, rz = pose_values
                pose_values=np.asarray(pose_values)
                
                if tool_pose_values_list==None:
                    tool_pose_values_list=pose_values
                else:
                    tool_pose_values_list=np.vstack((tool_pose_values_list,pose_values))

                            
           


                print(tool_pose_values_list)
        else:
            rospy.logwarn("AskItem to robot, but response not yet ok")
    
    except rospy.ServiceException as e:
        rospy.logerr("Error AskItem to robot: {}".format(e))
        return 1
    
    return 0

if __name__ == '__main__':
    main() """