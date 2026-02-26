#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import rclpy.waitable
from tm_msgs.srv import SendScript,SetIO,WriteItem
from tm_msgs.msg import FeedbackState 
import math
import numpy as np
import time
from std_msgs.msg import Int32
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point #float型態(x y z)
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32MultiArray
# import ROS2_gripper as rq
from aces_ws import ROS2_gripper as rq
import threading
import minimalmodbus
from time import sleep

class Moving_node(Node):
    # 定義一個繼承自 `Node` 的類 `ScriptSender`。

    def __init__(self,name):
        # 初始化方法，類似於 C++ 的構造函數。
        super().__init__(name)
        self.client_script = self.create_client(SendScript, 'send_script')
        self.client_IO = self.create_client(SetIO, 'set_io')
        self.client_write=self.create_client(WriteItem,'write_item')
        self.subscription=self.create_subscription(FeedbackState,'feedback_states',self.Feedback_state_call_back,10 ) # Queue size)
         # 訂閱 "feedback_states" 主題
        # self.subscription = self.create_subscription(FeedbackState,'feedback_states',self.tm_msg_callback,10)
        # 呼叫父類別的初始化方法，並設定節點名稱為 "demo_send_script"。
        self.renew_message_front_reverse_callback=None
        self.renew_message_Ry_angle_callback=None
        self.renew_message_python_point_callback=None

        self.renew_message_Point_place_callback=None
        self.renew_message_refill_position_callback=None

        self.latest_msg = None
        self.IO_state=None
        self.pose_x = None
        self.pose_y = None
        self.pose_Rz = None
        self.pose_Ry = None
        self.face = ""
        self.pose_y_err = 0.6092
        self.fixture_err_x_first = 0.3
        self.fixture_err_x_second= 0.2
        self.iter_xy = [13,20]
        self.photo_pose_1=[-0.28 , -554.94 , 75.41 , -178.93 , -0.81 , 92.30]
        self.photo_pose_2=[462.14 , -523.65 , 75.62 , 180.00 , 0.00 , -177.88 ]
        self.place_xy = []
        self.refill_position = []
        self.pick_total_count= 0  # 取了幾次料
        self.put_count= 0 # 轉盤上成功放了幾次料
# -------------------------------------------------------------------------------------------------訂閱部份
    # 初始化訂閱者
        self.subscription_front_reverse=self.create_subscription(String, 'front_reverse', self.front_reverse_callback, 10)
        self.subscription_robot_point_xy=self.create_subscription(Point, 'robot_point_xy', self.python_point_callback, 10)
        self.subscription_Ry_angle=self.create_subscription(Float64, 'Ry_angle', self.Ry_angle_callback, 10)
        self.subscription_Place_xy=self.create_subscription(Float32MultiArray, 'Place_xy', self.Point_place_callback, 10)
        self.subscription_refill=self.create_subscription(Float32MultiArray, 'refill', self.refill_position_callback, 10)

    # def wait_for_front_reverse(self):
    #     future = rclpy.Future()
    #     self.create_subscription(String, 'front_reverse', lambda msg: self.front_reverse_callback(msg, future), 10)
    #     rclpy.spin_until_future_complete(self, future)
    #     # return future.result().data

    # def wait_for_robot_point_xy(self):
    #     future = rclpy.Future()
    #     self.create_subscription(Point, 'robot_point_xy', lambda msg: self.python_point_callback(msg, future), 10)
    #     rclpy.spin_until_future_complete(self, future)
    #     # return future.result()

    # def wait_for_ry_angle(self):
    #     future = rclpy.Future()
    #     self.create_subscription(Float64, 'Ry_angle', lambda msg: self._message_callback(msg, future), 10)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().data
    
    # def wait_for_Place_xy(self):
    #     future = rclpy.Future()
    #     self.create_subscription(Float32MultiArray, 'Place_xy', self.Point_place_callback, 10)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()

    # def wait_for_refill(self):
    #     future = rclpy.Future()
    #     self.create_subscription(Float32MultiArray, 'refill', self.refill_position_callback, 10)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().data
    
    
    # ####################################################第一階段影像與點位訊息
    def front_reverse_callback(self, msg):
        self.face = msg.data
        self.get_logger().info(f"Front Reverse: {self.face}")
        self.renew_message_front_reverse_callback=True
        # if not future.done():
        #     future.set_result(self.face)

    

    def Ry_angle_callback(self, msg):
        self.pose_Ry = msg.data
        self.get_logger().info(f"Received Ry Angle: {self.pose_Ry}")
        self.renew_message_Ry_angle_callback=True
        # if not future.done():
        #     future.set_result(self.pose_Ry)

    def python_point_callback(self, msg):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_Rz = msg.z
        self.get_logger().info(f"Received Point - X: {self.pose_x}, Y: {self.pose_y}, Rz: {self.pose_Rz}")
        self.renew_message_python_point_callback=True
        
        # if not future.done():
        #     future.set_result((self.pose_x, self.pose_y, self.pose_Rz))
        
    # ####################################################
    

    
    # ####################################################第二階段影像與點位訊息
    def Point_place_callback(self, msg):
        place_xy = list(msg.data)
        if len(place_xy) % 2 != 0 or len(place_xy) < 20:
            self.get_logger().error("Invalid Place XY Points data length")
            return

        # 前10個為 x 座標，後10個為 y 座標
        x_coords = place_xy[:10]
        y_coords = place_xy[10:]

        # 使用 zip 將 x 和 y 配對成 (x, y) 的列表
        self.place_xy = list(zip(x_coords, y_coords))
        self.get_logger().info(f"Received Place XY Points: {self.place_xy}")
        self.renew_message_Point_place_callback=True

    def refill_position_callback(self, msg):
        self.refill_position = list(msg.data)
        self.get_logger().info(f"Received Refill Position: {self.refill_position}")
        self.renew_message_refill_position_callback=True

    def determine_pose_second_motion(self):
        if (self.pick_total_count<10):
            pick_pose=[57.1 , -483.22 , 100 , 180 , 0 , 180 ]
            return pick_pose
        else:
            pick_pose=[-92.91 , -482.23 , 100 , 180 , 0 , 180 ]
            return pick_pose
    # ####################################################



    def Feedback_state_call_back(self,msg):
        self.latest_msg=msg
        self.IO_state=msg.cb_digital_output
        # print(msg.cb_digital_output[0])
        # print(self.latest_msg)

    def get_IO(self,pin):
        rclpy.spin_once(self)
        return self.IO_state[pin]

# -------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------------------監測部份
    def IOcheck(self):
        """持續檢查 IO 狀態，每隔 0.1 秒等待並執行一次回呼處理"""
        while rclpy.ok():
            if self.get_IOsta():
                break
            rclpy.spin_once(self, timeout_sec=0.1)  # 每隔 0.1 秒等待一次

    def new_monitor(self,monitor_target_point):
        
        point_offset = 0.02
        arrive = 0

        # 使用變數來存儲接收到的訊息
        # self.latest_msg = None
        # latest_msg=self.latest_msg
        # print(latest_msg)

        """ def callback(msg):
            nonlocal latest_msg
            latest_msg = msg """

        """ # 創建訂閱者並指定回調函式
        subscription = node.create_subscription(
            FeedbackState,
            'feedback_states',
            callback,
            10  # Queue size
        )"""


        try:
            while rclpy.ok():
                # 執行一次訊息處理循環
                rclpy.spin_once(self, timeout_sec=0.1)
                # self.latest_msg = None
                latest_msg=self.latest_msg
                # print(latest_msg)


                if latest_msg is None:
                    self.get_logger().error("NO msg")
                    continue

                # 提取 tool_pose 資訊並轉換為數值列表
                if len(latest_msg.tool_pose) == 6:
                    robot_pose = [
                        latest_msg.tool_pose[0] * 1000,  # x msg unit: m
                        latest_msg.tool_pose[1] * 1000,  # y msg unit: m
                        latest_msg.tool_pose[2] * 1000,  # z msg unit: m
                        math.degrees(latest_msg.tool_pose[3]),  # rx msg unit: rad
                        math.degrees(latest_msg.tool_pose[4]),  # ry msg unit: rad
                        math.degrees(latest_msg.tool_pose[5])   # rz msg unit: rad
                    ]
                else:
                    self.get_logger().error("Invalid tool_pose length")
                    continue

                # 計算 robot_pose 與目標點位的變化程度
                change_of_point = abs((np.array(monitor_target_point[:3]) - np.array(robot_pose[:3])))
                
                # 檢查機器人是否到達目標點
                if all(abs(monitor_target_point[i] - robot_pose[i]) < point_offset for i in range(3)):
                    arrive += 1
                    self.get_logger().info(f"Change of point: {change_of_point[:3]}")
                else:
                    arrive = 0

                # 如果連續達到目標點三次，退出循環
                if arrive >= 3:
                    break

                # 重置 latest_msg 為 None，等待下一條訊息
                latest_msg = None

        finally:
            # self.destroy_subscription(self)
            print('Monitor Finish!')
# -------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------------------發送訊息部份
    def set_IO(self,pin=0,state=1.0,type=SetIO.Request.TYPE_DIGITAL_OUT):
        # 定義一個方法 `send_cmd`，用於發送指令。
        

                # 等待服務可用
        while not self.client_IO.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')  

        request = SetIO.Request()
        request.module = SetIO.Request.MODULE_CONTROLBOX
        request.type = type
        request.pin = pin
        request.state = state  # STATE_ON

        future = self.client_IO.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info('Service call succeeded: OK')
            else:
                self.get_logger().info('Service call succeeded: not OK')
        else:
            self.get_logger().error('Service call failed')

        # rclpy.shutdown()

    def write_item(self):
        request = WriteItem.Request()
        request.id = "detect"
        request.item = "g_complete_signal"
        request.value = "true"
        while not self.client_write.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("Service not available, waiting again...")
        
        future = self.client_write.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            if future.result().ok:
                self.get_logger().info("OK")
            else:
                self.get_logger().info("Not OK")
        else:
            self.get_logger().error("Failed to call service")    

    def move_send_script(self,*args,motion_type='Line', coordinate_type='CPP', speed=30, time=200, trajectory_percent=0, precision_arrive=False):
        # 定義一個方法 `send_cmd`，用於發送指令。
        

                # 等待服務可用
        while not self.client_script.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
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
        request = SendScript.Request()

        
        # 創建一個新的服務請求物件。


        request.id = 'demo'
        # 設定請求的 `id` 欄位為 "demo"。

        request.script = command
        # 將指令字串 `cmd` 賦值給請求的 `script` 欄位。
        

        future = self.client_script.call_async(request)
        # 發送非同步請求，並返回一個 future 對象。
        
        rclpy.spin_until_future_complete(self, future)
        # 等待請求完成。

        if future.result() is not None:
            # 如果請求成功完成，檢查結果。

            if future.result().ok:
                self.get_logger().info('OK')
                # 如果 `ok` 為 true，記錄成功信息。

            else:
                self.get_logger().info('Not OK')
                # 如果 `ok` 為 false，記錄失敗信息。

        else:
            self.get_logger().error('Service call failed')
            # 如果請求失敗，記錄錯誤信息。



    def leave_listen_node(self):
        # rclpy.init()  # 初始化rclpy
        # node = rclpy.create_node('leave_listen_node')  # 建立一個節點

        # 等待服務 'tm_driver/send_script' 可用
        self.get_logger().info('Waiting for service "Exit listen node"...')
        

        if not self.client_script.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service "Exit listen node" not available!')
            return

        # 創建服務請求
        request = SendScript.Request()
        request.id = "demo"  # 設定 ID
        request.script = "ScriptExit()"  # 設定指令為 ScriptExit()

        # 發送請求並等待回應
        future = self.client_script.call_async(request)

        # 等待回應
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result()}')
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')

        # node.destroy_node()  # 銷毀節點
        # rclpy.shutdown()  # 關閉 rclpy


    def change_tcp(self,your_script:str="ChangeTCP(\"11081\")")->None:
        self.get_logger().info('Waiting for service "Exit listen node"...')
        

        if not self.client_script.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service "Exit listen node" not available!')
            return

        # 創建服務請求
        request = SendScript.Request()
        request.id = "demo"  # 設定 ID
        request.script = your_script # 設定指令

        # 發送請求並等待回應
        future = self.client_script.call_async(request)

        # 等待回應
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result()}')
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')

    def gripper_pick(self,pick_distance:int,inital_bool:bool=True)->None:
        instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9)
        instrument.debug=False
        instrument.serial.baudrate = 115200
        myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB0',slaveaddress=9)

        if not inital_bool:
            myGripper.resetActivate()
            myGripper.calibrate(0, 255)
        myGripper.goTo(pick_distance, 90, 255)   #行程 速度 力量. 

# -------------------------------------------------------------------------------------------------
    def determine_pose(self):
        if self.face == 'reverse' and self.pose_Ry == 45:
            print("反面 +45 度")
            # 這裡是反面 +45 度的處理邏輯
            First_point = [56.70, -482.95, 120, -180, -45, 0]
            return First_point

        elif self.face == 'reverse' and self.pose_Ry == -45:
            print("反面 -45 度")
            # 這裡是反面 -45 度的處理邏輯
            First_point = [57.00, -483.20, 120, -180, 45, 180]
            return First_point

        elif self.face == 'front' and self.pose_Ry == -45:
            print("正面 +45 度")
            # 這裡是正面 -45 度的處理邏輯
            First_point = [56.30, -482.95, 120, -180, 45, 0]
            return First_point

        elif self.face == 'front' and self.pose_Ry == 45:
            print("正面 45 度")
            # 這裡是正面 +45 度的處理邏輯
            First_point = [56.85, -482.90, 120, -180, -45, 180]
            return First_point

        else:
            print("未定義的情況")



def main(args=None):

    
############################初始化##############################
    # 初始化 rclpy
    rclpy.init(args=args)

    # 創建節點物件
    node = Moving_node('move_to_fixture_node')

   
    
    node.change_tcp()
    node.gripper_pick(pick_distance=190,inital_bool=False)
    pallet_count_xy=[0,0] #散料至轉接 pallet控制
    pick_count_xy=[0,0] #轉接至端子 pallet控制
    time.sleep(2)
    node.leave_listen_node()
    node.new_monitor(node.photo_pose_1)



    # 使用SingleThreadedExecutor來確保回調能夠被處理
    # executor = SingleThreadedExecutor()
    # executor.add_node(node)
    # while rclpy.ok():
    # start_time = time.time()
    # timeout = 2.0  # 設定 2 秒超時

    # ####################################################第一階段運動
    while rclpy.ok():
        # break
        while node.renew_message_python_point_callback==None or node.renew_message_front_reverse_callback==None or node.renew_message_Ry_angle_callback==None:
            rclpy.spin_once(node)
        
        node.renew_message_python_point_callback=None
        node.renew_message_front_reverse_callback=None
        node.renew_message_Ry_angle_callback=None


        # node.wait_for_front_reverse()
        # node.wait_for_robot_point_xy()
        # print(node.face)
        # while rclpy.ok():
        #     rclpy.spin_once(node)
        
            
        
        # executor.spin_once(timeout_sec=0.01)
        fixture_point=[0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ]


        #1. 點位1,膠體正上方一段距離
        Scatter_Point=[node.pose_x,node.pose_y+node.pose_y_err,100,-180.00,node.pose_Ry,node.pose_Rz]
        node.move_send_script(Scatter_Point)
        node.new_monitor(Scatter_Point)

        # 2. 點位2,直下運動至膠體極近點
        Scatter_Point[2]=29
        node.move_send_script(Scatter_Point)
        node.new_monitor(Scatter_Point)
        
        # 3.夾取目標物
        node.gripper_pick(211)
        time.sleep(2)

        # 4. 點位3,將膠體抬起
        Scatter_Point[2]=140
        node.move_send_script(Scatter_Point)
        node.new_monitor(Scatter_Point)

        # 5. 根據不同姿態移動到轉接治具插槽上方一段距離

        # iter_errx=pallet_count_xy[1]*listener.fixture_err_x_first
        iter_distance_xy = [pallet_count_xy[0]*node.iter_xy[0]+pallet_count_xy[1]*node.fixture_err_x_first,
                            pallet_count_xy[1]*node.iter_xy[1]]

        
        Fixture_pose_origin=node.determine_pose()
        node.get_logger().info(f'Pose origin:{Fixture_pose_origin}')

        Fixture_pose_iter = [Fixture_pose_origin[0]+iter_distance_xy[0], Fixture_pose_origin[1]+iter_distance_xy[1], 120] + Fixture_pose_origin[3:]
        node.move_send_script(Fixture_pose_iter)
        node.new_monitor(Fixture_pose_iter)


        # 6. 距離插槽較為近的距離
        Fixture_pose_iter[2]=42
        node.move_send_script(Fixture_pose_iter)
        node.new_monitor(Fixture_pose_iter)

        # 7. 插入槽內
        Fixture_pose_iter[2]=34.3
        node.move_send_script(Fixture_pose_iter)
        node.new_monitor(Fixture_pose_iter)

        # 8. 張開一點夾抓放料
        node.gripper_pick(200)
        time.sleep(2)

        # 9. 回到插槽上方一段距離
        Fixture_pose_iter[2]=120
        node.move_send_script(Fixture_pose_iter)
        node.new_monitor(Fixture_pose_iter)

        # 10. 直行放料數值+1
        pallet_count_xy[1]+=1

        # 11. 判斷是否擺放滿10個膠體
        if (pallet_count_xy[0]==1 and pallet_count_xy[1]==5):
            pallet_count_xy[0] = 0
            pallet_count_xy[1] = 0
            break
        # 12. 否則換排繼續放料
        elif (pallet_count_xy[1]==5):
            pallet_count_xy[0] = 1
            pallet_count_xy[1] = 0

        # 13. 張開夾爪繼續取散料
        node.gripper_pick(180)
        time.sleep(0.3)

        # 14. 退出監聽節點
        node.leave_listen_node()

        # 15. 檢查是否到達拍照點位
        node.new_monitor(node.photo_pose_1)
        # rospy.loginfo('Complete first step!')
        # rospy.Duration(2)

    # ####################################################

    # 16. 退出節點 第一階段運動完成
    node.leave_listen_node()
    node.get_logger().info('First Motion Complete!')

    # ####################################################第二階段運動
    while rclpy.ok():
        # 17. 設置夾爪用於取轉接治具膠體
        node.gripper_pick(180)

         # 放完10次即跳出,並回到迴圈最開始
        # if (node.pick_total_count<10):
        if (node.put_count==10):
            node.put_count=0
            node.pick_total_count=0
            pick_count_xy=[0,0]
            node.write_item()
            
            break

        # 18. 取料點位設置與移動
        Fixture_pick_point_origin=node.determine_pose_second_motion()
        iter_distance_xy=[pick_count_xy[0]*node.iter_xy[0]+node.fixture_err_x_second*pick_count_xy[1],pick_count_xy[1]*node.iter_xy[1]]
        Fixture_pick_point_iter = [Fixture_pick_point_origin[0]+iter_distance_xy[0], Fixture_pick_point_origin[1]+iter_distance_xy[1], 100] + Fixture_pick_point_origin[3:]
        
        node.move_send_script(Fixture_pick_point_iter,motion_type='PTP')
        node.new_monitor(Fixture_pick_point_iter)

        # 19. 至轉接治具夾取膠體
        Fixture_pick_point_iter[2] = 35.7
        node.move_send_script(Fixture_pick_point_iter)
        node.new_monitor(Fixture_pick_point_iter)

        # 20. 夾爪夾取
        node.gripper_pick(211)
        time.sleep(2)

        # 21. 取料次數增加 1
        node.pick_total_count+=1
        node.get_logger().info(f'pick total count={node.pick_total_count}')

        # 22. 往取料點上方移動
        Fixture_pick_point_iter[2] = 100
        node.move_send_script(Fixture_pick_point_iter)
        node.new_monitor(Fixture_pick_point_iter)

        # 23. 退出監聽節點
        node.leave_listen_node()

        # 24. 監測是否抵達拍攝點位
        node.new_monitor(node.photo_pose_2)
        time.sleep(1)

        # 25. 接收插端子治具影像訊息
        while node.renew_message_Point_place_callback==None or node.renew_message_refill_position_callback==None:
            rclpy.spin_once(node)

        node.renew_message_Point_place_callback=None
        node.renew_message_refill_position_callback=None

        # 26. 檢測有無缺料
        if (node.put_count!=0):
            if(node.refill_position[node.put_count-1]==0):
                node.put_count-=1
                node.get_logger().warn(f"Detect of refill potion,Position={node.put_count};\n")

        # 27. 插端子治具位置之上方
        terminal_fixture_point=[node.place_xy[node.put_count][0] , node.place_xy[node.put_count][1] , 50 , 180 ,0 , 180]
        node.move_send_script(terminal_fixture_point)
        node.new_monitor(terminal_fixture_point)

        # 28. 端子治具插料點位
        terminal_fixture_point[2]=13.56
        node.move_send_script(terminal_fixture_point)
        node.new_monitor(terminal_fixture_point)

        # 29. 夾爪放料
        node.gripper_pick(200)
        time.sleep(2)

        # 30. 回到上方點位
        terminal_fixture_point[2]=50
        node.move_send_script(terminal_fixture_point)
        node.new_monitor(terminal_fixture_point)

       

        # 31. 流程參數計數
        pick_count_xy[1] += 1
        node.put_count += 1
        node.get_logger().info(f'put count ={node.put_count}')

        # 32. 換排
        if(pick_count_xy[1]== 5):
            pick_count_xy[0] = 1
            pick_count_xy[1] = 0
    
    print("exit")
        




    exit()    
  

if __name__ == '__main__':
    main()

