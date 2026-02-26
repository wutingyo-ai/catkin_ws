#!/usr/bin/env python3.10
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SetPositions, SetPositionsRequest,SendScript,SendScriptRequest,WriteItem,WriteItemRequest, SetIO,SetIORequest
import numpy as np
import math
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output,_Robotiq2FGripper_robot_input
from time import sleep
from std_msgs.msg import Int32,String,Float64,Float32MultiArray ,MultiArrayDimension
from geometry_msgs.msg import Point #float型態(x y z)
import threading
from typing import Union, List

pub_change_detect_method=rospy.Publisher("change_detect_method",Int32,queue_size = 10)

class Listener:
    def __init__(self):
        # 成員屬性初始化
        self.DI0 = False
        self.DO0 = False
        self.DO1 = True
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_Rz = 0.0
        self.pose_Ry = 0.0
        self.face = ""
        self.pose_y_err = 0.6092
        # self.fixture_err_x_first = 0.3 #Pallet的x向量有微小誤差
        self.fixture_err_x_first = 0.25 #Pallet的x向量有微小誤差
        self.fixture_err_x_second= 0.2 #Pallet的x向量有微小誤差
        self.iter_xy = [13,20]

        self.photo_pose_1=[-0.28 , -554.94 , 75.41 , -178.93 , -0.81 , 92.30] #TCP 11081之下
        self.photo_pose_2=[471.98 , -398.99 , 76.15 , 179.99 , 0.01 , -175.42 ]#TCP 11081之下

        # self.photo_pose_1=[-0.58, -554.88 , 63.95 , -178.93 , -0.81 , 92.30] #TCP WVS之下
        # self.photo_pose_2=[462.23 , -523.72 , 64.16 , 180.00 , 0.00 , -177.88 ]#TCP WVS之下

        self.place_xy = []
        self.refill_position = []
        self.pick_total_count= 0  # 手臂取了幾次料
        self.put_count= 0 # 轉盤上成功放了幾次料
        self.fixture1_count=0 #轉接治具有多少料
        self.fixture2_count=10
        self.client_IO = rospy.ServiceProxy('tm_driver/set_io', SetIO)  # 假設服務名稱為 'set_io
        

    # def getIOsta(self):
    #     """返回 IO 狀態，不可更改"""
    #     return self.IOsta
    
    

    def get_IO_sta(self):
        # 等待並接收 "feedback_states" 主題的訊息
        msg = rospy.wait_for_message("feedback_states", FeedbackState)
        self.DI0 = msg.cb_digital_input[0]
        rospy.loginfo(f"pin={self.DI0}")
        # rospy.loginfo("Received feedback states: %s", msg)
        # return msg
    
    def IOcheck(self):
        """持續檢查 IO 狀態，每隔 0.1 秒等待並執行一次回呼處理"""
        while not rospy.is_shutdown():
            self.get_IO_sta()
            if self.DI0:
                print(f'DI0={self.DI0}')
                break
            rospy.sleep(0.1)  # 每隔 0.1 秒等待一次
    # def TMmsgCallback(self, msg):
    #     # 獲取 FeedbackState 的 cb_digital_input[0] 值k
        

    def fetch_front_reverse(self):
        # 膠體正反面消息
        msg = rospy.wait_for_message("front_reverse", String)
        self.face = msg.data
        rospy.loginfo("Front Reverse: %s", self.face)

    def fetch_python_point(self):
        # 膠體散料區 X,Y點位置，還有一RZ礁體旋轉方向 Point=(X,Y,Rz)
        msg = rospy.wait_for_message("robot_point_xy", Point)
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_Rz = msg.z  # 假設 z 對應 pose_Rz
        rospy.loginfo("Received Point - X: %f, Y: %f, Rz: %f", self.pose_x, self.pose_y, self.pose_Rz)

    def fetch_Ry_angle(self):
        # 等待接收 'Ry_angle' 消息對應至開口方向朝上或下
        msg = rospy.wait_for_message("Ry_angle", Float64)
        self.pose_Ry = msg.data
        rospy.loginfo("Received Ry Angle: %f", self.pose_Ry)

    # def fetch_Point_place(self):
    #     # 插端子治具([x1,y1,x2,y2....,x10,y10]) 各點位陣列
    #     msg = rospy.wait_for_message("Place_xy", Float32MultiArray)
    #     self.place_xy = list(msg.data)
    #     rospy.loginfo("Received Place XY Points: %s", str(self.place_xy))

    def fetch_Point_place(self):
        # 插端子治具([x1,y1,x2,y2....,x10,y10]) 各點位陣列
        msg = rospy.wait_for_message("Place_xy", Float32MultiArray)
        place_xy = list(msg.data)
        
        # 檢查數據長度是否符合期望（必須是偶數，且至少包含10對座標）
        if len(place_xy) % 2 != 0 or len(place_xy) < 20:
            rospy.logerr("Invalid Place XY Points data length")
            return

        # 前10個為 x 座標，後10個為 y 座標
        x_coords = place_xy[:10]
        y_coords = place_xy[10:]
        
        # 使用 zip 將 x 和 y 配對成 (x, y) 的列表
        coordinates = list(zip(x_coords, y_coords))

        self.place_xy=coordinates
        rospy.loginfo("Received Place XY Points: %s", str(self.place_xy))

    


    def fetch_refill_position(self):
        # [0,1,0...]後檢測放置情形
        msg = rospy.wait_for_message("refill", Float32MultiArray)
        self.refill_position = list(msg.data)
        rospy.loginfo("Received Refill Position: %s", str(self.refill_position))

    def start_fetching_first_step(self):
        # 建立和啟動線程
        front_reverse_thread = threading.Thread(target=self.fetch_front_reverse)
        python_point_thread = threading.Thread(target=self.fetch_python_point)
        Ry_angle_thread = threading.Thread(target=self.fetch_Ry_angle)

        front_reverse_thread.start()
        python_point_thread.start()
        Ry_angle_thread.start()

        # 等待線程完成
        front_reverse_thread.join()
        python_point_thread.join()
        Ry_angle_thread.join()

    def determine_pose_fixture1(self):
        if self.face == 'reverse' and self.pose_Ry == 45:
            print("反面 +45 度")
            # 這裡是反面 +45 度的處理邏輯
            First_point = [55.47, -484.07, 120, -180, -45, 0]  # 11081座標系
            # First_point = [56.70, -482.95, 120, -180, -45, 0]  # 11081座標系
            # First_point = [56.70, -482.95, 120, -180, -45, 0]  # WVS座標系
            return First_point
    
        elif self.face == 'reverse' and self.pose_Ry == -45:
            print("反面 -45 度")
            # 這裡是反面 -45 度的處理邏輯
            First_point = [56.41, -483.94, 120, -180, 45, 180]  # 11081座標系
            # First_point = [57.00, -483.20, 120, -180, 45, 180]  # 11081座標系
            # First_point = [57.00, -483.80, 120, -180, 45, 180]  # WVS座標系
            return First_point
    
        elif self.face == 'front' and self.pose_Ry == -45:
            print("正面 -45 度")
            # 這裡是正面 -45 度的處理邏輯
            First_point = [56.41, -483.45, 120, -180, 45, 0]  # 11081座標系
            # First_point = [57.02, -482.95, 120, -180, 45, 0]  # 11081座標系
            # First_point = [56.30, -482.95, 120, -180, 45, 0]  # WVS座標系
            return First_point
    
        elif self.face == 'front' and self.pose_Ry == 45:
            print("正面 +45 度")
            # 這裡是正面 +45 度的處理邏輯
            First_point = [57.12, -483.52, 120, -180, -45, 180]  # 11081座標系
            # First_point = [56.85, -482.90, 120, -180, -45, 180]  # 11081座標系
            # First_point = [56.85, -483.88, 120, -180, -45, 180]  # WVS座標系
            return First_point
    
        else:
            print("未定義的情況")
    
    
    def determine_pose_fixture2(self):
        if self.face == 'reverse' and self.pose_Ry == 45:
            print("反面 +45 度")
            # 這裡是反面 +45 度的處理邏輯
            First_point = [6.71, -482.95, 120, -180, -45, 0]  # 11081座標系
            # First_point = [56.70, -482.95, 120, -180, -45, 0]  # WVS座標系
            return First_point
    
        elif self.face == 'reverse' and self.pose_Ry == -45:
            print("反面 -45 度")
            # 這裡是反面 -45 度的處理邏輯
            First_point = [7.30, -482.94, 120, -180, 45, 180]  # 11081座標系
            # First_point = [57.00, -483.80, 120, -180, 45, 180]  # WVS座標系
            return First_point
    
        elif self.face == 'front' and self.pose_Ry == -45:
            print("正面 -45 度")
            # 這裡是正面 -45 度的處理邏輯
            First_point = [6.58, -482.91, 120, -180, 45, 0]  # 11081座標系
            # First_point = [56.30, -482.95, 120, -180, 45, 0]  # WVS座標系
            return First_point
    
        elif self.face == 'front' and self.pose_Ry == 45:
            print("正面 +45 度")
            # 這裡是正面 +45 度的處理邏輯
            First_point = [6.78, -482.91, 120, -180, -45, 180]  # 11081座標系
            # First_point = [56.85, -483.88, 120, -180, -45, 180]  # WVS座標系
            return First_point
    
        else:
            print("未定義的情況")
    # def determine_pose_fixture1(self):
    #     match (self.face, self.pose_Ry):
    #         case ('reverse', 45): #ok
    #             print("反面 +45 度")
    #             # 這裡是反面 +45 度的處理邏輯
    #             First_point = [55.47,-484.07,120,-180,-45,0]#11081座標系
    #             # First_point = [56.70,-482.95,120,-180,-45,0]#11081座標系
    #             # First_point = [56.70,-482.95,120,-180,-45,0]#WVS座標系
    #             return First_point
            
    #         case ('reverse', -45): #ok
    #             print("反面 -45 度")
    #             # 這裡是反面 -45 度的處理邏輯
    #             First_point = [56.41,-483.94,120,-180,45,180]#11081座標系
    #             # First_point = [57.00,-483.20,120,-180,45,180]#11081座標系
    #             # First_point = [57.00,-483.80,120,-180,45,180]#WVS座標系
    #             return First_point
        
    #         case ('front', -45): #ok
    #             print("正面 -45 度")
    #             # 這裡是正面 -45 度的處理邏輯
    #             First_point = [56.41,-483.45,120,-180,45,0] #11081座標系
    #             # First_point = [57.02,-482.95,120,-180,45,0] #11081座標系
    #             # First_point = [56.30,-482.95,120,-180,45,0] #WVS座標系
    #             return First_point
            
    #         case ('front', 45): #ok
    #             print("正面 +45 度")
    #             # 這裡是正面 +45 度的處理邏輯
    #             First_point = [57.12,-483.52,120,-180,-45,180] #11081座標系
    #             # First_point = [56.85,-482.90,120,-180,-45,180] #11081座標系
    #             # First_point = [56.85,-483.88,120,-180,-45,180] #WVS座標系
    #             return First_point
            
    #         case _:
    #             print("未定義的情況")
    
    # def determine_pose_fixture2(self):
    #     match (self.face, self.pose_Ry):
    #         case ('reverse', 45):
    #             print("反面 +45 度")
    #             # 這裡是反面 +45 度的處理邏輯
    #             First_point = [6.71,-482.95,120,-180,-45,0]#11081座標系
    #             # First_point = [56.70,-482.95,120,-180,-45,0]#WVS座標系
    #             return First_point
            
    #         case ('reverse', -45):
    #             print("反面 -45 度")
    #             # 這裡是反面 -45 度的處理邏輯
    #             First_point = [7.30,-482.94,120,-180,45,180]#11081座標系
    #             # First_point = [57.00,-483.80,120,-180,45,180]#WVS座標系
    #             return First_point
        
    #         case ('front', -45):
    #             print("正面 -45 度")
    #             # 這裡是正面 -45 度的處理邏輯
    #             First_point = [6.58,-482.91,120,-180,45,0] #11081座標系
    #             # First_point = [56.30,-482.95,120,-180,45,0] #WVS座標系
    #             return First_point
            
    #         case ('front', 45):
    #             print("正面 +45 度")
    #             # 這裡是正面 +45 度的處理邏輯
    #             First_point = [6.78,-482.91,120,-180,-45,180] #11081座標系
    #             # First_point = [56.85,-483.88,120,-180,-45,180] #WVS座標系
    #             return First_point
            
    #         case _:
    #             print("未定義的情況")

    def determine_pose_second_motion(self):
        if (self.fixture1_count<=10 and self.fixture1_count>0):
            # pick_pose=[57.1 , -483.22 , 100 , 180 , 0 , 180 ]
            # pick_pose=[57.3 , -483.22 , 100 , 180 , 0 , 180 ]
            # pick_pose=[57.3 , -483.47 , 100 , 180 , 0 , 180 ]
            pick_pose=[56.45 , -483.47 , 100 , 180 , 0 , 180 ]
            # pick_pose=[57.4 , -483.47 , 100 , 180 , 0 , 179 ]
            # pick_pose=[57.3 , -483.47 , 100 , 180 , 0 , 180 ]
            self.fixture1_count-=1
            rospy.loginfo('\033[34m' + f'fixture1_count={self.fixture1_count}' + '\033[0m')
            return pick_pose
        
        elif (self.fixture1_count==0 and self.fixture2_count<=10 and self.fixture2_count>0  ):
            pick_pose=[7.28 , -483.02 , 100 , 180 , 0 , 180 ]
            self.fixture2_count-=1
            rospy.loginfo('\033[92m' + f'fixture2_count={self.fixture2_count}' + '\033[0m')
            return pick_pose
        
    def start_fetching_second_step(self):
        # 建立和啟動線程
        fetch_Point_place_thread = threading.Thread(target=self.fetch_Point_place)
        fetch_refill_position_thread = threading.Thread(target=self.fetch_refill_position)
        

        fetch_Point_place_thread.start()
        fetch_refill_position_thread.start()

        # 等待線程完成
        fetch_Point_place_thread.join()
        fetch_refill_position_thread.join()
        
    def set_IO(self, pin=0, state=1.0, type=SetIORequest.TYPE_DIGITAL_OUT):
        """發送指令以設置 IO 狀態"""
        # 等待服務可用
        

        request = SetIORequest()
        request.module = SetIORequest.MODULE_CONTROLBOX
        request.type = type
        request.pin = pin
        request.state = state  # STATE_ON
        
        # Call the service
        try:
            resp = self.client_IO(request)
            if resp.ok:
                
                rospy.loginfo("SetIO to robot")
                # robot_pose=monitor.start_monitoring()
                # rospy.loginfo("current pose="+str(robot_pose))
            else:
                rospy.logwarn("SetIO to robot, but response not yet ok")
        except rospy.ServiceException as e:
            rospy.logerr("Error SetIO to robot: %s", e)
            return 1

def new_monitor(monitor_target_point:list)->list:
    rate = rospy.Rate(100)
    arrive = 0
    # monitor_target_point=construct_point(monitor_target_point)
    point_offset=0.02
    while rospy.is_shutdown()==False:
        # 使用 wait_for_message 直接獲取最新的 FeedbackState 訊息
        msg = rospy.wait_for_message('feedback_states', FeedbackState)
        
        # 提取 tool_pose 資訊並轉換為數值列表
        if len(msg.tool_pose) == 6:
            robot_pose = [
                msg.tool_pose[0]*1000,  # x msg unit:m
                msg.tool_pose[1]*1000,  # y msg unit:m
                msg.tool_pose[2]*1000,  # z msg unit:m
                math.degrees(msg.tool_pose[3]),  # rx msg unit:rad
                math.degrees(msg.tool_pose[4]),  # ry msg unit:rad
                math.degrees(msg.tool_pose[5])   # rz msg unit:rad
            ]
            # print(robot_pose)
        else:
            rospy.logerr("Invalid tool_pose length")
            continue

        # 計算 robot_pose 與目標點位的變化程度
        change_of_point = abs((np.array(monitor_target_point[:3]) - np.array(robot_pose[:3])))
        # print(robot_pose[:3])
        
        if abs(monitor_target_point[0]-robot_pose[0])<point_offset and abs(monitor_target_point[1]-robot_pose[1])<point_offset and abs(monitor_target_point[2]-robot_pose[2])<point_offset:
            arrive+=1
            rospy.loginfo(str(change_of_point[:3]))
            
        elif abs(monitor_target_point[0]-robot_pose[0])<point_offset and abs(monitor_target_point[1]-robot_pose[1])<point_offset and abs(monitor_target_point[2]-robot_pose[2])<point_offset:
            arrive=0
        
        if arrive>=3:
            # rospy.loginfo(str(change_of_point[:3]))
            break

        

        rate.sleep()
        # print('change of point =', change_of_point)
    
    print('Monitor Finish!')

    # return robot_pose

# The get_gripper_position function is to get the current gripper distance.
def get_gripper_position()->int:
    """接收一次夾爪位置訊息並返回 gPO 值。"""
    try:
        # 等待並接收一次 Robotiq2FGripperRobotInput topic 的訊息
        msg = rospy.wait_for_message("Robotiq2FGripperRobotInput", _Robotiq2FGripper_robot_input.Robotiq2FGripper_robot_input)
        # 返回夾爪位置
        return msg.gPO
    except rospy.ROSException:
        rospy.logwarn("Failed to receive gripper position message.")
        return None

""" 2F gripper status interpreter
-----
gACT = 1: Gripper activation
gGTO = 1: Go to Position Request
gSTA = 3: Activation is completed
gOBJ = 3: Fingers are at requested position
gFLT = 0: No Fault
gPR = 211: Echo of the requested position for the Gripper: 211/255
gPO = 211: Position of Fingers: 211/255
gCU = 0: Current of Fingers: 0 mA """

# The construct_point function is to create the target point which can be used by the function 'approach' and 'go_to_position'.
# The parameter is x y z(unit:m) ;rx ry rz(unit:degree) in euler angle.
def construct_point(*args: Union[float, List[float]]) -> List[float]:
    # 檢查是否傳入一個包含 xyz 和 rx, ry, rz 的 list
    if len(args) == 1 and isinstance(args[0], list) and len(args[0]) == 6:
        x, y, z, rx, ry, rz = args[0]
    elif len(args) == 6:
        x, y, z, rx, ry, rz = args
    else:
        raise ValueError("請傳入6個浮點數或一個包含6個元素的列表")

    # 將超過 1 的 x, y, z 縮放為毫米單位
    x = x / 1000 if abs(x) > 1 else x
    y = y / 1000 if abs(y) > 1 else y
    z = z / 1000 if abs(z) > 1 else z

    # 構建目標點的列表，並將角度轉換為弧度
    target_point = [x, y, z, math.radians(rx), math.radians(ry), math.radians(rz)]
    return target_point


# The Motion_types have four format:PTP_J=1 (point to poin in joint), PTP_T=2 (point to poin in cartensian), LINE_J=3 (Line in joint), LINE_T=4 (Line in cartesian) 
# blend_percentage is from 0~100 % 
# accelerate_time (sec) 
# The go to position will directly move to the target point.
def go_to_position(target_point:construct_point,Motion_type:int=4,velocity:float=30,accelerate_time:float=1,blend_percentage:int=100)->None:
    # monitor=Monitor(target_point)
    # Create a service client for 'tm_driver/set_positions'
    # rospy.wait_for_service('tm_driver/set_positions')
    client = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)

    # Prepare the request
    req = SetPositionsRequest()
    req.motion_type = Motion_type #SetPositionsRequest.PTP_J

    # req.positions = [0, 0, 1.58, 0, 1.58, 0]
    # req.velocity = 0.4  # rad/s
    # req.acc_time = 0.2

    # rad_target_point=[x/rad_to_degree for x in target_point]
    # req.positions =rad_target_point

    req.positions =target_point


    req.velocity = velocity  # joint rad/s
    req.acc_time = accelerate_time
    
    req.blend_percentage = blend_percentage
    req.fine_goal = False

    # Call the service
    try:
        resp = client(req)
        if resp.ok:
            
            rospy.loginfo("SetPositions to robot")
            # robot_pose=monitor.start_monitoring()
            # rospy.loginfo("current pose="+str(robot_pose))
        else:
            rospy.logwarn("SetPositions to robot, but response not yet ok")
    except rospy.ServiceException as e:
        rospy.logerr("Error SetPositions to robot: %s", e)
        return 1

    # rospy.loginfo("Shutdown.")
    # return 0


# Change the coordinate from orgignal to '11081',which is the tool tcp.
def change_tcp(your_script:str="ChangeTCP(\"11081\")")->None:
    # 创建服务客户端
    client = rospy.ServiceProxy('tm_driver/send_script', SendScript)

    # 等待服务可用
    # client.wait_for_service()

    try:
        # 构建请求
        request = SendScriptRequest()
        request.script = your_script

        # 调用服务
        response = client(request)

        # 检查是否成功
        if response.ok:
            rospy.loginfo("TCP changed successfully.")
        else:
            rospy.logwarn("Failed to change TCP.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

# The gripper_pick function will let robotiq 2f gripper to act.
# In the first time we using, need to set the intial_bool=False.So that the gripper can be initalized and act.
# The pick_distance can set the gripper to close or open.The arange is from 0 ~255 in integer.The 255 will fully close the gripper,and 0 will fully open.
def gripper_pick(pick_distance:int,inital_bool:bool=True)->None:
    pub_gripper = rospy.Publisher('/Robotiq2FGripperRobotOutput', _Robotiq2FGripper_robot_output.Robotiq2FGripper_robot_output, queue_size=1)
    gripper_command = _Robotiq2FGripper_robot_output.Robotiq2FGripper_robot_output()
    if inital_bool==False:

        rospy.sleep(2)
        pub_gripper.publish(gripper_command)
        rospy.sleep(2)
        #####################
        
        gripper_command.rACT = 1
        gripper_command.rATR = 0
        gripper_command.rFR = 150
        gripper_command.rGTO = 1
        gripper_command.rPR = 0
        gripper_command.rSP = 255
        pub_gripper.publish(gripper_command)
        rospy.sleep(2)
    gripper_command.rACT = 1
    gripper_command.rATR = 0
    gripper_command.rFR = 150
    gripper_command.rGTO = 1
    gripper_command.rPR = pick_distance
    gripper_command.rSP = 20
    pub_gripper.publish(gripper_command)
    rospy.sleep(1)
    print('PICK')



# Initialize the ros node
def init_ros_node()->None:
    rospy.init_node('Main_node')    
    
def leave_listen_node():
    # 等待服務 'tm_driver/send_script' 可用
    # rospy.wait_for_service('tm_driver/send_script')

    try:
        # 創建服務代理
        send_script_client = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        
        # 創建服務請求
        request = SendScriptRequest()
        request.id = "demo"  # 設定 ID
        request.script = "ScriptExit()"  # 設定指令為 ScriptExit()

        # 發送請求並獲取回應
        response = send_script_client(request)
        rospy.loginfo("Service call successful: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def vision(vision_type):
    # 等待服務 'tm_driver/send_script' 可用
    # rospy.wait_for_service('tm_driver/send_script')
    if vision_type==1:
        vision_name="11_06"
    elif vision_type==2:
        vision_name="Second_motion_1118"
        
    try:
        # 創建服務代理      
        send_script_client = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        
        # 創建服務請求
        request = SendScriptRequest()
        request.id = "vision"  # 設定 ID
        request.script = f'Vision_DoJob_PTP("{vision_name}", 100, 250, false)'  # 設定指令

        # 發送請求並獲取回應
        response = send_script_client(request)
        rospy.loginfo("Service call successful: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def write_item():
    # rospy.wait_for_service('tm_driver/write_item')
    client = rospy.ServiceProxy('tm_driver/write_item', WriteItem)

    # Prepare the request
    req = WriteItemRequest()
    req.id = "detect"
    req.item ="g_complete_signal"
    req.value = "true"

    # Call the service
    try:
        resp = client(req)
        if resp.ok:
            
            rospy.loginfo("Write item to robot")
        else:
            rospy.logwarn("Write item to robot, but response not yet ok")
    except rospy.ServiceException as e:
        rospy.logerr("Error Write item to robot: %s", e)
        return 1

def move_send_script(*args,motion_type='Line', coordinate_type='CPP', speed=100, time=200, trajectory_percent=0, precision_arrive=False):
    # 如果是列表則解包
    if len(args) == 1 and isinstance(args[0], list) and len(args[0]) == 6:
        x, y, z, rx, ry, rz = args[0]
    elif len(args) == 6:
        # 否則將每個單一值賦給 x, y, z, rx, ry, rz
        x, y, z, rx, ry, rz = args
    else:
        raise ValueError("Invalid arguments: Please provide either a list of six values or six separate values for position.")

    # 等待服務 'tm_driver/send_script' 可用
    # rospy.wait_for_service('tm_driver/send_script')
    
    # 格式化並生成所需的字符串
    command = f'{motion_type}("{coordinate_type}",{x},{y},{z},{rx},{ry},{rz},{speed},{time},{trajectory_percent},{precision_arrive})'

    try:
        # 創建服務代理
        send_script_client = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        
        # 創建服務請求
        request = SendScriptRequest()
        request.id = "demo"  # 設定 ID
        request.script = command  # 設定指令

        # 發送請求並獲取回應
        response = send_script_client(request)
        rospy.loginfo(f'pose of [{x},{y},{z}]')
        rospy.loginfo("Sendscript call successful: %s\n", response)
        # new_monitor([x,y,z,rx,ry,rz])

    except rospy.ServiceException as e:
        rospy.logerr("Sendscript call failed: %s", e)

def change_detect_method(type:int,publisher=pub_change_detect_method): #切換不同影像偵測方法
    publisher.publish(type)


# The example to paln a simple task.
def main():
 
#############初始化###############
    rospy.init_node('main_node')
    # new_monitor([-165.47,-524.90,362.52,-159.51,36.03,115.07])
    # return 0
    
    gripper_pick(pick_distance=180,inital_bool=False)

    print('gripper finish')
    
    control_speed=100
    
    listener=Listener()
    
    # listener.get_IO_sta()
    # print(type(listener.IOsta))
    # exit()
    fixture1_pallet_count_xy=[0,0] #散料至轉接1 pallet控制
    fixture2_pallet_count_xy=[0,0] #散料至轉接2 pallet控制

    pick_count_xy=[0,0] #轉接至端子 pallet控制
    change_tcp()
    # change_tcp(your_script="ChangeTCP(\"11081\")")
    rospy.sleep(0.3)
    # listener.set_IO()
    
    
    # input("wait")

    # leave_listen_node()
    while rospy.is_shutdown()==False:
            
    
        # if input("是否要進行一輪? y繼續")=="y":
            
        # #################################################### 散料至第一轉接治具
            while rospy.is_shutdown()==False:
                #檢查DI0訊號是否到達為true
                # break
                # listener.IOcheck()
                print(f'listener.fixture_count={listener.fixture1_count}')

                listener.set_IO(pin=0,state=1)
                print(f'listener.DO')
                # break
                if(listener.fixture1_count==1):
                    fixture1_pallet_count_xy[0]=0
                    fixture1_pallet_count_xy[1]=0
                    break


                change_detect_method(type=1)
                vision(vision_type=1)
                # 使用多線程來並行接收消息
                listener.start_fetching_first_step()

                listener.fixture1_count+=1

                #1. 點位1,膠體正上方一段距離
                Scatter_Point=[listener.pose_x, listener.pose_y+listener.pose_y_err, 100, -180.00, listener.pose_Ry,  listener.pose_Rz]
                move_send_script(Scatter_Point)
                new_monitor(Scatter_Point)

                # 2. 點位2,直下運動至膠體極近點

                Scatter_Point[2]=33 #原始深度
                # Scatter_Point[2]=29 #原始深度
                # Scatter_Point[2]=35 #空夾測試

                # move_send_script(Scatter_Point,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                move_send_script(Scatter_Point)
                new_monitor(Scatter_Point)

                # 3.夾取目標物
                gripper_pick(211)
                rospy.sleep(2)

                # 4. 點位3,將膠體抬起
                Scatter_Point[2]=140
                move_send_script(Scatter_Point)
                new_monitor(Scatter_Point)

                # 5. 根據不同姿態移動到轉接治具插槽上方一段距離

                # iter_errx=pallet_count_xy[1]*listener.fixture_err_x_first
                iter_distance_xy = [fixture1_pallet_count_xy[0]*listener.iter_xy[0]+fixture1_pallet_count_xy[1]*listener.fixture_err_x_first, #x 迭代
                                    fixture1_pallet_count_xy[1]*listener.iter_xy[1]] #y迭代
                
                Fixture_pose_origin=listener.determine_pose_fixture1()
                rospy.loginfo(Fixture_pose_origin)

                Fixture_pose_iter = [Fixture_pose_origin[0]+iter_distance_xy[0], Fixture_pose_origin[1]+iter_distance_xy[1], 120] + Fixture_pose_origin[3:]
                move_send_script(Fixture_pose_iter)
                new_monitor(Fixture_pose_iter)
                
                # 6. 距離插槽較為近的距離
                Fixture_pose_iter[2]=42
                # move_send_script(Fixture_pose_iter,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                move_send_script(Fixture_pose_iter,speed=control_speed)
                new_monitor(Fixture_pose_iter)
                
                # 7. 插入槽內
                Fixture_pose_iter[2]=34.3 #原始深度
                # Fixture_pose_iter[2]=41 #空夾測試
                # move_send_script(Fixture_pose_iter,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                move_send_script(Fixture_pose_iter,speed=control_speed)
                new_monitor(Fixture_pose_iter) 

                # 8. 張開一點夾抓放料
                gripper_pick(200)
                rospy.sleep(2)

                # 9. 回到插槽上方一段距離
                Fixture_pose_iter[2]=120
                move_send_script(Fixture_pose_iter)
                new_monitor(Fixture_pose_iter)
                
                # 10. 直行放料數值+1
                fixture1_pallet_count_xy[1]+=1

                # 11. 判斷是否擺放滿10個膠體
                if (fixture1_pallet_count_xy[0]==1 and fixture1_pallet_count_xy[1]==5):
                    fixture1_pallet_count_xy[0] = 0
                    fixture1_pallet_count_xy[1] = 0
                    break
                # 12. 否則換排繼續放料
                elif (fixture1_pallet_count_xy[1]==5):
                    fixture1_pallet_count_xy[0] = 1
                    fixture1_pallet_count_xy[1] = 0
                
                # 13. 張開夾爪繼續取散料
                gripper_pick(180)
                rospy.sleep(0.3)
                # input("Stop")

                # 14. 退出監聽節點
                # leave_listen_node()

                # 15. 檢查是否到達拍照點位
                # new_monitor(listener.photo_pose_1)
                # rospy.loginfo('Complete first step!')
                # rospy.Duration(2)
                
        # ####################################################散料至第二轉接治具
        
            if (listener.fixture2_count<10):
                gripper_pick(180)
                while rospy.is_shutdown()==False:
                    # break
                    if(listener.fixture2_count==10):
                        fixture2_pallet_count_xy[0]=0
                        fixture2_pallet_count_xy[1]=0
                        break


                    change_detect_method(type=1)
                    vision(vision_type=1)
                    # 使用多線程來並行接收消息
                    listener.start_fetching_first_step()

                    listener.fixture2_count+=1

                    #1. 點位1,膠體正上方一段距離
                    Scatter_Point=[listener.pose_x, listener.pose_y+listener.pose_y_err, 100, -180.00, listener.pose_Ry,  listener.pose_Rz]
                    move_send_script(Scatter_Point)
                    new_monitor(Scatter_Point)

                    # 2. 點位2,直下運動至膠體極近點

                    Scatter_Point[2]=29 #原始深度
                    # Scatter_Point[2]=35 #空夾測試

                    # move_send_script(Scatter_Point,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                    move_send_script(Scatter_Point)
                    new_monitor(Scatter_Point)

                    # 3.夾取目標物
                    gripper_pick(211)
                    rospy.sleep(2)

                    # 4. 點位3,將膠體抬起
                    Scatter_Point[2]=140
                    move_send_script(Scatter_Point)
                    new_monitor(Scatter_Point)

                    # 5. 根據不同姿態移動到轉接治具插槽上方一段距離

                    # iter_errx=pallet_count_xy[1]*listener.fixture_err_x_first
                    iter_distance_xy = [fixture2_pallet_count_xy[0]*listener.iter_xy[0]+fixture2_pallet_count_xy[1]*listener.fixture_err_x_first, #x 迭代
                                        fixture2_pallet_count_xy[1]*listener.iter_xy[1]] #y迭代
                    
                    Fixture_pose_origin=listener.determine_pose_fixture2()
                    rospy.loginfo(Fixture_pose_origin)

                    Fixture_pose_iter = [Fixture_pose_origin[0]+iter_distance_xy[0], Fixture_pose_origin[1]+iter_distance_xy[1], 120] + Fixture_pose_origin[3:]
                    move_send_script(Fixture_pose_iter)
                    new_monitor(Fixture_pose_iter)
                    
                    # 6. 距離插槽較為近的距離
                    Fixture_pose_iter[2]=42
                    # move_send_script(Fixture_pose_iter,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                    move_send_script(Fixture_pose_iter,speed=control_speed)
                    new_monitor(Fixture_pose_iter)
                    
                    # 7. 插入槽內
                    Fixture_pose_iter[2]=34.3 #原始深度
                    # Fixture_pose_iter[2]=41 #空夾測試
                    # move_send_script(Fixture_pose_iter,coordinate_type='CAP',speed=control_speed) #,coordinate_type='CAP',speed=control_speed
                    move_send_script(Fixture_pose_iter,speed=control_speed)
                    new_monitor(Fixture_pose_iter) 

                    # 8. 張開一點夾抓放料
                    gripper_pick(200)
                    rospy.sleep(2)

                    # 9. 回到插槽上方一段距離
                    Fixture_pose_iter[2]=120
                    move_send_script(Fixture_pose_iter)
                    new_monitor(Fixture_pose_iter)
                    
                    # 10. 直行放料數值+1
                    fixture2_pallet_count_xy[1]+=1

                    # 11. 判斷是否擺放滿10個膠體
                    if (fixture2_pallet_count_xy[0]==1 and fixture2_pallet_count_xy[1]==5):
                        fixture2_pallet_count_xy[0] = 0
                        fixture2_pallet_count_xy[1] = 0
                        break
                    # 12. 否則換排繼續放料
                    elif (fixture2_pallet_count_xy[1]==5):
                        fixture2_pallet_count_xy[0] = 1
                        fixture2_pallet_count_xy[1] = 0
                    
                    # 13. 張開夾爪繼續取散料
                    gripper_pick(180)
                    rospy.sleep(0.3)
                    # input("Stop")

            # 16. 退出節點 第一階段運動完成
            # input("Stop")
            # leave_listen_node()
            rospy.loginfo('pick and place complete!')
            
            # listener.fixture1_count=10
        # ####################################################插端子治具
            while rospy.is_shutdown()==False:
                # 17. 設置夾爪用於取轉接治具膠體
                
                gripper_pick(180)
                
                # 放完10次即跳出,並回到迴圈最開始
                if (listener.put_count==1):
                        listener.put_count=0
                        listener.pick_total_count=0
                        pick_count_xy=[0,0]

                        listener.set_IO(pin=0,state=0)
                        listener.set_IO(pin=1,state=1)
                        print(f'reset DO0 DO1')
                        rospy.sleep(1)
                        # write_item()
                        # leave_listen_node()
                        break
                elif (listener.fixture2_count<5):
                    break
                
                
                print(f'listener.DI0={listener.DI0}')
                # 18. 取料點位設置與移動，槽點正上方
                # iter_errx=listener.fixture_err_x_second*pick_count_xy[1]
                Fixture_pick_point_origin=listener.determine_pose_second_motion()
                iter_distance_xy=[pick_count_xy[0]*listener.iter_xy[0]+listener.fixture_err_x_second*pick_count_xy[1],pick_count_xy[1]*listener.iter_xy[1]]
                Fixture_pick_point_iter = [Fixture_pick_point_origin[0]+iter_distance_xy[0], Fixture_pick_point_origin[1]+iter_distance_xy[1], 100] + Fixture_pick_point_origin[3:]

                move_send_script(Fixture_pick_point_iter,motion_type='PTP')
                new_monitor(Fixture_pick_point_iter)

                # 19. 至轉接治具槽點夾取膠體
                Fixture_pick_point_iter[2] = 38
                move_send_script(Fixture_pick_point_iter)
                new_monitor(Fixture_pick_point_iter)

                # 20. 夾爪夾取
                gripper_pick(211)
                rospy.sleep(2)

                # 21. 取料次數增加 1
                listener.pick_total_count+=1
                # listener.fixture1_count-=1
                
                rospy.loginfo(f'pick total count={listener.pick_total_count}')

                # 22. 往取料點上方移動
                Fixture_pick_point_iter[2] = 100
                move_send_script(Fixture_pick_point_iter)
                new_monitor(Fixture_pick_point_iter)

                # 23. 退出監聽節點
                # leave_listen_node()

                # 24. 監測是否抵達拍攝點位
                # new_monitor(listener.photo_pose_2)
                change_detect_method(type=2)
                vision(vision_type=2)
                
                print("接收端子治具訊息")
                # 25. 接收插端子治具影像訊息
                listener.start_fetching_second_step()

                # 26. 檢測有無缺料
                if (listener.put_count!=0):
                    if(listener.refill_position[listener.put_count-1]==0):
                        listener.put_count-=1
                        rospy.logwarn(f"Detect of refill potion,Position={listener.put_count};\n")

                # 27. 插端子治具位置之上方
                # terminal_fixture_point=[listener.place_xy[listener.put_count][0] , listener.place_xy[listener.put_count][1] , 50 , 180 , 0 , 180]
                terminal_fixture_point=[listener.place_xy[listener.put_count][0] , listener.place_xy[listener.put_count][1] , 50 , 179.99 , 0.00 , -175.42]
                
                move_send_script(terminal_fixture_point)
                new_monitor(terminal_fixture_point)

                # 28. 端子治具插料點位
                # terminal_fixture_point[2]=13.56
                terminal_fixture_point[2]=25.55
                move_send_script(terminal_fixture_point,speed=control_speed)
                new_monitor(terminal_fixture_point)

                # 29. 夾爪放料
                gripper_pick(200)
                rospy.sleep(2)

                # 30. 回到上方點位
                terminal_fixture_point[2]=50
                move_send_script(terminal_fixture_point)
                new_monitor(terminal_fixture_point)

                # 31. 流程參數計數
                pick_count_xy[1] += 1
                listener.put_count += 1
                rospy.loginfo(f'put count ={listener.put_count}')
                
                # 32. 換排
                

                if(pick_count_xy[1]== 5 and pick_count_xy[0]==1):
                    pick_count_xy[0] = 0
                    pick_count_xy[1] = 0
                    

                elif(pick_count_xy[1]== 5):
                    pick_count_xy[0] = 1
                    pick_count_xy[1] = 0
        
        # else:
            # break
            


        

    



if __name__ == '__main__':
    main()




    '''
    rospy.init_node('demo_set_positions')
        sub = rospy.Subscriber('feedback_states', FeedbackState, tm_msg_callback, queue_size=10)
        rate = rospy.Rate(2)
        
        while not rospy.is_shutdown():
            
            rospy.spinOnce()
            rate = rospy.Rate(2)
            
            # if change_of_point<1:
            #     break
    '''
        
    """ while not rospy.is_shutdown():
            rospy.spinOnce()
            rate.sleep() """

    
""" 
def gripper(pub_gripper,distance):
    # pub_gripper = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=1)
    gripper_command = Robotiq2FGripper_robot_output()
    gripper_command.rACT = 1  # 啟動夾爪
    gripper_command.rATR = 0
    gripper_command.rFR = 150
    gripper_command.rGTO = 1
    gripper_command.rPR = distance  # 夾爪行程
    gripper_command.rSP = 20
    pub_gripper.publish(gripper_command)
    sleep(1)

def gripper_initialization(pub_gripper):
    # 初始化 ROS 節點
    # pub_gripper = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=1)
    gripper_command = Robotiq2FGripper_robot_output()
    gripper_command.rACT = 0
    gripper_command.rATR = 0
    gripper_command.rFR = 0
    gripper_command.rGTO = 0
    gripper_command.rPR = 0
    gripper_command.rSP = 0
    pub_gripper.publish(gripper_command)
    sleep(1)
    
    gripper_command.rACT = 1
    gripper_command.rATR = 0
    gripper_command.rFR = 150
    gripper_command.rGTO = 1
    gripper_command.rPR = 0
    gripper_command.rSP = 255
    pub_gripper.publish(gripper_command)
    sleep(1)
     """


""" class Monitor:
    def __init__(self, monitor_target_point):
        self.monitor_target_point = np.array(monitor_target_point)
        self.robot_pose = None
        self.arrive = 0
        # self.total_arrive=0

        # 訂閱 'feedback_states' topic，並將回調函數設為 self.feedback_callback
        self.subscriber = rospy.Subscriber('feedback_states', FeedbackState, self.feedback_callback)

    def feedback_callback(self, msg):
        # 提取 tool_pose 資訊並轉換為數值列表
        if len(msg.tool_pose) == 6:
            self.robot_pose = np.array([
                msg.tool_pose[0],  # x
                msg.tool_pose[1],  # y
                msg.tool_pose[2],  # z
                msg.tool_pose[3],  # rx
                msg.tool_pose[4],  # ry
                msg.tool_pose[5]   # rz
            ])

            # 計算 robot_pose 與目標點位的變化程度
            change_of_point = abs(np.sum(self.monitor_target_point - self.robot_pose))

            if self.arrive >= 2:
                rospy.loginfo("Arrived at target point")
                self.arrive=0

               
            elif change_of_point < 0.01:
                self.arrive += 1

            if self.arrive > 2:
                rospy.loginfo(f'Arrive count = {self.arrive}')
                rospy.loginfo(f'Change of point = {change_of_point}')
        else:
            rospy.logerr("Invalid tool_pose length")

    def start_monitoring(self):
        rate = rospy.Rate(100)
        arrive=0
        while not rospy.is_shutdown() and arrive < 2:
            # 持續運行，並等待回調函數來更新 arrive 狀態
            arrive=self.arrive
            rate.sleep()
            # print(self.arrive)

        return self.robot_pose """



# rad_to_degree = 57.2958

'''robot_pose=[] #the robot pose which accept from the tm_msg_callback
def monitor(monitor_target_point):
    global robot_pose
    # rospy.init_node('demo_set_positions')
    sub = rospy.Subscriber('feedback_states', FeedbackState, tm_msg_callback, queue_size=1)
    rate = rospy.Rate(10)
    arrive=0
    while rospy.is_shutdown()==False:
        
        # rospy.spin()
        rate.sleep()
        # change_of_point=abs(sum((monitor_target_point-list(robot_pose))))
        change_of_point = abs(np.sum(np.array(monitor_target_point) - np.array(robot_pose)))
        if arrive>=5:
            break
        elif change_of_point<0.01  :
            arrive+=1
        print('arrive=',arrive) 
        print('change of point=',change_of_point)   

            

    """ while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep() """

    return 0
def tm_msg_callback(msg):
    global robot_pose
    if len(msg.tool_pose) == 6:



        # robot_pose=msg.tool_pose[:]
        robot_pose = [
            msg.tool_pose[0],  # x
            msg.tool_pose[1],  # y
            msg.tool_pose[2],  # z
            msg.tool_pose[3],  # rx  rad
            msg.tool_pose[4],  # ry  rad
            msg.tool_pose[5]   # rz  rad
        ]
        """ rospy.loginfo("FeedbackState: tool pos = (%fm, %fm, %fm, %.2fdegree, %.2fdegree, %.2fdegree)",
                     robot_pose[0],
                     robot_pose[1], 
                     robot_pose[2],
                     robot_pose[3]*rad_to_degree, 
                     robot_pose[4]*rad_to_degree,
                     robot_pose[5]*rad_to_degree) """
        """ rospy.loginfo("tcp force -> FX=%.2f;FY=%.2f;FZ=%.2f",
                     msg.tcp_force[0], msg.tcp_force[1], msg.tcp_force[2]) """
    else:
        rospy.logerr("Error FeedbackState callback")
'''




