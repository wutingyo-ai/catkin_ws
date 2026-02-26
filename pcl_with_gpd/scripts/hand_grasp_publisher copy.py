#!/usr/bin/env python3.8
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob
import os

""" header: 
  seq: 0
  stamp: 
    secs: 1743151426
    nsecs: 388361593
  frame_id: "base_link"
grasps: 
  - 
    position:                 夾爪底座點位
      x: 0.012828796474861612
      y: -0.13561316932808215
      z: 0.4783507410607037
    approach:                 夾爪x軸
      x: 0.08924355164381441
      y: 0.43458607916271447
      z: 0.8961978176094698
    binormal:                 夾爪y軸
      x: -0.4571055319199846
      y: -0.781563079589023
      z: 0.42451582457132325
    axis:                     夾爪z軸
      x: 0.884923793994797
      y: -0.4475422800376534
      z: 0.1289022358244915
    width: 
      data: 0.0577111579477787
    score: 
      data: 128.28512573242188
    sample:                 要夾取點位
      x: 0.0204689372330904
      y: -0.10563018918037415
      z: 0.5300000309944153
---
 """


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




######################################################################################
# 假設這是夾爪的姿態資訊
grasp_position = np.array([0.039376347448636016, -0.10646306254295379, 0.5181931234131114])
# grasp_orientation = np.array([0.08924355164381441, 0.43458607916271447, 0.8961978176094698])  # 這裡可以是夾爪的 approach 向量

#夾取點位
sample=np.array([0.04996556043624878,-0.12076372653245926,0.5320000052452087])

# 夾爪的 x, y, z 軸向量
approach = np.array([0.35788252344131627, -0.2707350217232367, 0.8936568958094568])  # x 軸
binormal = np.array([-0.3313232776759314, 0.8579551018660015, 0.39260403570541225])  # y 軸
axis = np.array([-0.8730091552127923, -0.43659545484878515, 0.21734632207614876])  # z 軸

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
if(rz>=0 and rz<= 90):
  rz-=180
elif(rz>=-90 and rz<=0):
 rz+=180
 
print(rx,ry,rz)
exit()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
      
      
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





