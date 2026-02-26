#!/usr/bin/env python3.8
import rospy
from gpd_ros.msg import GraspConfigList
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob
import os
from mpl_toolkits.mplot3d import Axes3D

R_cam2base=np.array(
[[-0.020,0.905,-0.425],
[0.995,0.059,0.078],
[0.096,-0.422,-0.902]])

t_cam2base=np.array(
[[0.940],
[-0.139],
[0.623]])

T_cam2base=np.array(
[[-0.020,0.905,-0.425,0.940],
[0.995,0.059,0.078,-0.139],
[0.096,-0.422,-0.902,0.623],
[0.000,0.000,0.000,1.000]])

rs_camera_intrinsic=np.array(
                            [[908.2088623,  0,         644.47814941 ],
                            [  0,       907.76373291 ,361.23849487],
                            [  0,           0,           1        ]])

# 定義轉移矩陣的函數
def R_and_t_to_T(R, t):
    T = np.hstack((R, t.reshape(-1, 1)))  # 將平移向量轉換為列向量
    T = np.vstack((T, [0, 0, 0, 1]))  # 添加最後一行
    return T

def T_to_R_and_t(T):
    Rt = T[:3]
    R = Rt[:, :3]
    t = Rt[:, 3].reshape((-1, 1))
    return R, t
  
  

 
 
 
# 全域變數
grasp_in_camera_list=[]
T_grasps_in_base_list = []
T_tool_list=[]

def grasp_callback(msg):
    global T_grasps_in_base_list,T_grasps_in_base_list,T_tool_list  # 使用全域變數
    
    
    rospy.loginfo("收到 %d 個抓取姿態", len(msg.grasps))
    
    for i, grasp in enumerate(msg.grasps):
        # 將抓取姿態的資訊轉換為 np.array
        position = np.array([grasp.position.x, grasp.position.y, grasp.position.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        axis = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
        sample = np.array([grasp.sample.x, grasp.sample.y, grasp.sample.z]) 
        
        # print(R_cam2base)

        # 輸出轉換後的數據
        # rospy.loginfo("位置: %s", position)
        # rospy.loginfo("接近向量: %s", approach)
        # rospy.loginfo("法向量: %s", binormal)
        # rospy.loginfo("軸向量: %s", axis)
        # rospy.loginfo("樣本點: %s", sample)
        # 建立旋轉矩陣
        R_grasp = np.column_stack((approach, binormal, axis))

        # 建立轉移矩陣
        T_grasp = R_and_t_to_T(R_grasp, position) 
        grasp_in_camera_list.append(T_grasp)
        # 夾爪在 base 座標系下的轉移矩陣
        T_grasp_in_base = T_cam2base @ T_grasp

        # 將座標系繪製的數據存儲起來
        T_grasps_in_base_list.append(T_grasp_in_base)

        # 若要分離出旋轉與平移部分（可選）
        R_grasp_in_base,t_grasp_in_base=T_to_R_and_t(T_grasp_in_base)


        # 將 sample 點轉成齊次座標
        sample_homog = np.append(sample, 1)  # [x, y, z, 1]
        print(sample_homog)
        # 將 sample 點從 camera frame 轉換到 base frame
        sample_in_base = T_cam2base @ sample_homog
        x,y,z,_=sample_in_base*1000
        # print('sample=',x,y,z)


        # 再將姿態轉成對應的工具座標系定義
        # TCP_frame_transform=np.array(
        # [[0,0,1,0],
        # [0,-1,0,0],
        # [1,0,0,0],
        # [0,0,0,1]])
        
        TCP_frame_transform=np.array(
        [[0,1,0,0],
        [0,0,1,0],
        [1,0,0,0],
        [0,0,0,1]])
        
        TCP_frame_transform_inv=np.linalg.inv(TCP_frame_transform)
        
        
        T_tool=  T_grasp_in_base@ TCP_frame_transform_inv
        T_tool_list.append(T_tool)
        R_tool,t_tool=T_to_R_and_t(T_tool)

        euler_angles=R_scipy.from_matrix(R_tool).as_euler('xyz',True)
        rx,ry,rz=euler_angles
        # if(rz>=0 and rz<= 90):
        #     rz-=180
        # elif(rz>=-90 and rz<=0):
        #     rz+=180
        # grasps_select_list_tm.append(np.array([x, y, z, rx, ry, rz], dtype=float))
        print(f'x={x:.3f},y={y:.3f},z={z:.3f},rx={rx:.3f},ry={ry:.3f},rz={rz:.3f}')
        print('score=%f' % float(grasp.score.data))
        print('width=%f'% float(grasp.width.data))
        print(len(T_grasps_in_base_list))
        
        
        
        






""" ######################################################################################
# 假設這是夾爪的姿態資訊
grasp_position = np.array([0.012828796474861612, -0.13561316932808215, 0.4783507410607037])
# grasp_orientation = np.array([0.08924355164381441, 0.43458607916271447, 0.8961978176094698])  # 這裡可以是夾爪的 approach 向量

#夾取點位
sample=np.array([0.0204689372330904,-0.10563018918037415,0.5300000309944153])

# 夾爪的 x, y, z 軸向量
approach = np.array([0.08924355164381441, 0.43458607916271447, 0.8961978176094698])  # x 軸
binormal = np.array([-0.4571055319199846, -0.781563079589023, 0.42451582457132325])  # y 軸
axis = np.array([0.884923793994797, -0.4475422800376534, 0.1289022358244915])  # z 軸

# 建立旋轉矩陣
R_grasp = np.column_stack((approach, binormal, axis))

#建立轉移矩陣
T_grasp= R_and_t_to_T(R_grasp,grasp_position) 

# 夾爪在 base 座標系下的轉移矩陣
T_grasp_in_base = T_cam2base @ T_grasp

# 印出來確認
# print("夾爪在 base 座標系下的轉移矩陣：")
# print(T_grasp_in_base)

# 若要分離出旋轉與平移部分（可選）
R_grasp_in_baser,t_grasp_in_base=T_to_R_and_t(T_grasp_in_base)


# 將 sample 點轉成齊次座標
sample_homog = np.append(sample, 1)  # [x, y, z, 1]

# 將 sample 點從 camera frame 轉換到 base frame
sample_in_base = T_cam2base @ sample_homog
x,y,z,_=sample_in_base*1000
print('sample=',x,y,z)


# 再將姿態轉成對應的工具座標系定義
TCP_frame_transform=np.array(
[[0,0,1,0],
[0,-1,0,0],
[1,0,0,0],
[0,0,0,1]])

T_tool=T_grasp_in_base @ TCP_frame_transform
R_tool,t_tool=T_to_R_and_t(T_tool)

euler_angles=R_scipy.from_matrix(R_tool).as_euler('xyz',True)
rx,ry,rz=euler_angles

print(rx,ry,rz) """

def draw_coordinate_system(T, base_origin, label=''):
    """ 繪製座標系 """
    # 提取旋轉矩陣和位移向量
    R = T[:3, :3]
    t = T[:3, 3]

    # 繪製基座座標系
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 基座座標系的原點
    ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
              1, 0, 0, color='r', label='Base X', length=0.1)
    ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
              0, 1, 0, color='g', label='Base Y', length=0.1)
    ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
              0, 0, 1, color='b', label='Base Z', length=0.1)

    # 繪製夾爪座標系
    ax.quiver(t[0], t[1], t[2], 
              R[0, 0], R[1, 0], R[2, 0], 
              color='c', label=f'{label} X', length=0.1)
    ax.quiver(t[0], t[1], t[2], 
              R[0, 1], R[1, 1], R[2, 1], 
              color='m', label=f'{label} Y', length=0.1)
    ax.quiver(t[0], t[1], t[2], 
              R[0, 2], R[1, 2], R[2, 2], 
              color='y', label=f'{label} Z', length=0.1)

    # 設定圖形屬性
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title(f'{label} Coordinate System')
    ax.legend()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    # plt.pause(0.001)  # 使用非阻塞的方式更新圖形
    plt.show()
    # plt.draw()  # 更新圖形
    # plt.pause(0.001)  # 確保圖形能夠更新

def draw_all_coordinate_systems(T_list):
    """ 繪製所有抓取姿態的座標系 """
    draw_coordinate_system(T_list[len(T_list)-1], base_origin=np.array([0, 0, 0]), label='Grasp')
    # for T in T_tool_list:
    #     if T.ndim == 2 and T.shape == (4, 4):  # 確保是 4x4 矩陣
    #         draw_coordinate_system(T, base_origin=np.array([0, 0, 0]), label='Grasp')
    #     else:
    #         rospy.logwarn("無效的轉移矩陣，跳過繪製。")

def main():
    global T_tool_list
    rospy.init_node("grasp_subscriber", anonymous=True)
    
    # 訂閱 /detect_grasps/clustered_grasps 話題
    # rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, grasp_callback)
    
    rospy.loginfo("抓取姿態訂閱者節點已啟動，等待消息...")
    
    # plt.ion()  # 開啟互動模式
    
    while not rospy.is_shutdown():
        # 等待接收一條消息
        msg = rospy.wait_for_message("/detect_grasps/clustered_grasps", GraspConfigList)
        grasp_callback(msg)  # 處理接收到的消息
        draw_all_coordinate_systems(T_tool_list)  # 繪製所有座標系

if __name__ == "__main__":
    main()
  

exit()

print("夾取點在 base 座標系下的位置:", sample_in_base[:3])

print(T_grasp)
exit()



# 將旋轉矩陣與平移向量組合成轉移矩陣
T_grasp = np.eye(4)
T_grasp[:3, :3] = R_grasp
T_grasp[:3, 3] = grasp_position  # 使用之前的夾爪位置

# 定義從 screw 到 camera 的平移和旋轉
translation_screw_to_camera = np.array([0.032, 0.013, -0.010])


# 計算從 screw 到 camera 的旋轉矩陣
quaternion_screw_to_camera = np.array([0.499, -0.500, 0.500, 0.501])
rotation_screw_to_camera = R_scipy.from_quat(quaternion_screw_to_camera).as_matrix() 

# 建立從 screw 到 camera 的轉移矩陣 (4x4)
T_screw_to_camera = np.eye(4)
T_screw_to_camera[:3, :3] = rotation_screw_to_camera
T_screw_to_camera[:3, 3] = translation_screw_to_camera

""" # 定義從 camera 到 base 的旋轉矩陣和平移
R_cam2base = np.array(
[[0.932, 0.126, 0.339],
[-0.357, 0.164, 0.920],
[0.060, -0.978, 0.198]])

t_cam2base = np.array([[-0.360], [-0.875], [0.132]]) """

# 建立從 camera 到 base 的轉移矩陣 (4x4)
T_camera_to_base = np.eye(4)
T_camera_to_base[:3, :3] = R_cam2base
T_camera_to_base[:3, 3] = t_cam2base.flatten()

# 計算從 screw 到 base 的轉移矩陣
T_screw_to_base = T_camera_to_base @ T_screw_to_camera
R_screw_to_base = T_screw_to_base[:3, :3]
t_screw_to_base = T_screw_to_base[:3, 3]

# 輸出結果
print("從camera_rgb_frame 到 base 的轉移矩陣 ")
print(T_camera_to_base)

print("從 screw 到 base 的轉移矩陣:")
print(T_screw_to_base)

# 將旋轉矩陣轉換為歐拉角 (XYZ)
rotation = R_scipy.from_matrix(R_screw_to_base)
euler_angles = rotation.as_euler('xyz', degrees=True)

# 輸出位移和旋轉角度
print(f"平移向量 (XYZ): {t_screw_to_base}")
print(f"旋轉角度radians (RX, RY, RZ): {np.radians(euler_angles)}")

# 在這裡你可以發布 TF 轉換訊息
# 例如，使用 rospy 發布 TF 訊息的範例
# import rospy
# import tf
# broadcaster = tf.TransformBroadcaster()
# broadcaster.sendTransform(
#     t_screw_to_base,
#     rotation.as_quat(),
#     rospy.Time.now(),
#     "base",
#     "screw"
# )

# 繪製座標系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 基座座標系的原點
base_origin = np.array([0, 0, 0])

# 繪製 base 座標系 (紅色)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          1, 0, 0, color='r', label='Base X', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 1, 0, color='g', label='Base Y', length=0.5)
ax.quiver(base_origin[0], base_origin[1], base_origin[2], 
          0, 0, 1, color='b', label='Base Z', length=0.5)

# 繪製 cam 螺絲底座 座標系
cam_bottom_screw_origin = t_screw_to_base
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 0], R_screw_to_base[1, 0], R_screw_to_base[2, 0], 
          color='r', label='Cam X', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 1], R_screw_to_base[1, 1], R_screw_to_base[2, 1], 
          color='g', label='Cam Y', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_screw_to_base[0, 2], R_screw_to_base[1, 2], R_screw_to_base[2, 2], 
          color='b', label='Cam Z', length=0.5)

# 繪製 camera 到 base 的座標系
camera_rgb_frame_origin= t_cam2base
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_cam2base[0, 0], R_cam2base[1, 0], R_cam2base[2, 0], 
          color='c', label='Camera X', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_cam2base[0, 1], R_cam2base[1, 1], R_cam2base[2, 1], 
          color='m', label='Camera Y', length=0.5)
ax.quiver(cam_bottom_screw_origin[0], cam_bottom_screw_origin[1], cam_bottom_screw_origin[2], 
          R_cam2base[0, 2], R_cam2base[1, 2], R_cam2base[2, 2], 
          color='y', label='Camera Z', length=0.5)

# 設定圖形屬性
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Base and Camera Coordinate Systems')
ax.legend()
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

plt.show()



""" 平移向量 (XYZ): [-0.331928 -0.893492  0.119226]
旋轉角度 (RX, RY, RZ): [ -3.63431022 -11.30655766  69.75749912] """


""" 2025/03/28
R_cam2base=
np.array(
[[-0.020,0.905,-0.425],
[0.995,0.059,0.078],
[0.096,-0.422,-0.902]])

t_cam2base=
np.array(
[[0.940],
[-0.139],
[0.623]])

T_cam2base=
np.array(
[[-0.020,0.905,-0.425,0.940],
[0.995,0.059,0.078,-0.139],
[0.096,-0.422,-0.902,0.623],
[0.000,0.000,0.000,1.000]]) """
""" 2025/03/28
R_cam2base=
np.array(
[[-0.020,0.905,-0.425],
[0.995,0.059,0.078],
[0.096,-0.422,-0.902]])

t_cam2base=
np.array(
[[0.940],
[-0.139],
[0.623]])

T_cam2base=
np.array(
[[-0.020,0.905,-0.425,0.940],
[0.995,0.059,0.078,-0.139],
[0.096,-0.422,-0.902,0.623],
[0.000,0.000,0.000,1.000]]) """

