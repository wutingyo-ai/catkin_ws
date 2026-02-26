import rclpy
# 引入 ROS 2 的 Python 介面。

from rclpy.node import Node
# 引入節點類，提供建立和操作 ROS 2 節點的功能。

from tm_msgs.srv import SendScript
# 引入服務訊息類型 `SendScript`，這是 ROS 2 服務的定義。

import sys
# 引入 `sys` 模組，用於處理命令行參數。

from typing import Union, List

class ScriptSender(Node):
    # 定義一個繼承自 `Node` 的類 `ScriptSender`。

    def __init__(self,name):
        # 初始化方法，類似於 C++ 的構造函數。
        super().__init__(name)
        # 呼叫父類別的初始化方法，並設定節點名稱為 "demo_send_script"。

        self.client = self.create_client(SendScript, 'send_script')
        # 創建一個 `SendScript` 服務的客戶端，目標服務名稱為 "send_script"。

        while not self.client.wait_for_service(timeout_sec=1.0):
            # 等待服務可用，每次等待 1 秒。
            self.get_logger().info('Service not available, waiting again...')
            # 如果服務不可用，記錄信息並重試。

    def send_cmd(self, *args,motion_type='Line', coordinate_type='CPP', speed=30, time=200, trajectory_percent=0, precision_arrive=False):
        # 定義一個方法 `send_cmd`，用於發送指令。
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

        future = self.client.call_async(request)
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




def main(args=None):
    # 主函數入口，啟動程式。

    rclpy.init(args=args)
    # 初始化 rclpy。

    script_sender = ScriptSender('move_send_script_node')
    # 創建一個 `ScriptSender` 節點物件。

    # cmd = 'PTP("JPP",0,0,90,0,90,0,35,200,0,false)'
    # 定義要發送的指令。

    script_sender.send_cmd([204.229,-388.181,509.985,154.090,-11.500,-0.312])
    # 調用 `send_cmd` 方法發送指令。



    
    rclpy.shutdown()
    # 關閉並清理 rclpy。

if __name__ == '__main__':
    main()
    # 如果此文件被執行，調用 `main` 函數。



# def move_send_script(*args,motion_type='Line', coordinate_type='CPP', speed=30, time=200, trajectory_percent=0, precision_arrive=False):
#     # 如果是列表則解包
#     if len(args) == 1 and isinstance(args[0], list) and len(args[0]) == 6:
#         x, y, z, rx, ry, rz = args[0]
#     elif len(args) == 6:
#         # 否則將每個單一值賦給 x, y, z, rx, ry, rz
#         x, y, z, rx, ry, rz = args
#     else:
#         raise ValueError("Invalid arguments: Please provide either a list of six values or six separate values for position.")

#     # 等待服務 'tm_driver/send_script' 可用
#     # rospy.wait_for_service('tm_driver/send_script')
    
#     # 格式化並生成所需的字符串
#     command = f'{motion_type}("{coordinate_type}",{x},{y},{z},{rx},{ry},{rz},{speed},{time},{trajectory_percent},{precision_arrive})'

#     try:
#         # 創建服務代理
#         send_script_client = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        
#         # 創建服務請求
#         request = SendScriptRequest()
#         request.id = "demo"  # 設定 ID
#         request.script = command  # 設定指令

#         # 發送請求並獲取回應
#         response = send_script_client(request)
#         rospy.loginfo("Sendscript call successful: %s\n", response)
#         # new_monitor([x,y,z,rx,ry,rz])

#     except rospy.ServiceException as e:
#         rospy.logerr("Sendscript call failed: %s", e)
