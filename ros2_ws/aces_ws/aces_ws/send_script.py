import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from tm_msgs.msg import FeedbackState 
import math
import numpy as np
# def send_cmd(cmd, node):
#     # 創建一個服務客戶端，目標服務為 "send_script"
#     client = node.create_client(SendScript, 'send_script')

#     # 等待服務可用
#     while not client.wait_for_service(timeout_sec=1.0):
#         node.get_logger().info('Service not available, waiting again...')

#     # 創建一個新的服務請求
#     request = SendScript.Request()
#     request.id = 'demo'  # 設定請求的 ID
#     request.script = cmd  # 設定要發送的腳本

#     # 發送非同步請求，並等待結果
#     future = client.call_async(request)

#     # 等待請求完成
#     rclpy.spin_until_future_complete(node, future)

#     if future.result() is not None:
#         # 如果請求成功，檢查結果的 `ok` 欄位
#         if future.result().ok:
#             node.get_logger().info('OK')
#         else:
#             node.get_logger().info('Not OK')
#     else:
#         # 如果請求失敗，記錄錯誤
#         node.get_logger().error('Service call failed')


def send_cmd(*args,motion_type='Line', coordinate_type='CPP', speed=30, time=200, trajectory_percent=0, precision_arrive=False,node=None):
        # 定義一個方法 `send_cmd`，用於發送指令。
        client = node.create_client(SendScript, 'send_script')

                # 等待服務可用
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting again...')
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
        

        future = client.call_async(request)
        # 發送非同步請求，並返回一個 future 對象。

        rclpy.spin_until_future_complete(node, future)
        # 等待請求完成。

        if future.result() is not None:
            # 如果請求成功完成，檢查結果。

            if future.result().ok:
                node.get_logger().info('OK')
                # 如果 `ok` 為 true，記錄成功信息。

            else:
                node.get_logger().info('Not OK')
                # 如果 `ok` 為 false，記錄失敗信息。

        else:
            node.get_logger().error('Service call failed')
            # 如果請求失敗，記錄錯誤信息。

def new_monitor(monitor_target_point,node):
    
    point_offset = 0.008
    arrive = 0

    # 使用變數來存儲接收到的訊息
    latest_msg = None

    def callback(msg):
        nonlocal latest_msg
        latest_msg = msg

    # 創建訂閱者並指定回調函式
    subscription = node.create_subscription(
        FeedbackState,
        'feedback_states',
        callback,
        10  # Queue size
    )

    try:
        while rclpy.ok():
            # 執行一次訊息處理循環
            rclpy.spin_once(node, timeout_sec=0.1)

            if latest_msg is None:
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
                node.get_logger().error("Invalid tool_pose length")
                continue

            # 計算 robot_pose 與目標點位的變化程度
            change_of_point = abs((np.array(monitor_target_point[:3]) - np.array(robot_pose[:3])))

            # 檢查機器人是否到達目標點
            if all(abs(monitor_target_point[i] - robot_pose[i]) < point_offset for i in range(3)):
                arrive += 1
                node.get_logger().info(f"Change of point: {change_of_point[:3]}")
            else:
                arrive = 0

            # 如果連續達到目標點三次，退出循環
            if arrive >= 3:
                break

            # 重置 latest_msg 為 None，等待下一條訊息
            latest_msg = None

    finally:
        node.destroy_subscription(subscription)
    print('Monitor Finish!')





def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)

    # 創建節點物件
    node = Node('demo_send_script_node')

    # 定義要發送的指令
    P1=[204.229,-388.181,509.985,154.090,-11.500,-0.312]
    P2=[-296.725,-388.171,311.528,154.090,-11.500,-0.312]
    # 發送指令
    send_cmd(P1,node=node)
    new_monitor(P1,node=node)
    send_cmd(P2,node=node)
    new_monitor(P2,node=node)
    
    node.destroy_node()  
    # 清理並關閉 rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()
