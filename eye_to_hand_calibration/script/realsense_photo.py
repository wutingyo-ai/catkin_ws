#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2
import os

pipeline = rs.pipeline()
config = rs.config()
pipeline.start(config)
pc = rs.pointcloud()

image_count = 0
SAVE_PATH = "/home/chen/eye_to_hand_sample"

def Get_RGB_Frame():
    frame = pipeline.wait_for_frames()
    color_frame = frame.get_color_frame()
    return np.asanyarray(color_frame.get_data())

def callback(data):
    global image_count
    global SAVE_PATH
    if data.data == 'ok':
        rospy.loginfo("Received 'ok' message. Capturing image.")
        rgb_image = Get_RGB_Frame()
        
        # 轉換顏色空間從BGR到RGB
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
       # 確保保存路徑存在
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        
        # 保存圖像
        filename = os.path.join(SAVE_PATH, f"{image_count}.png")
        cv2.imwrite(filename, rgb_image)
        rospy.loginfo(f"Image saved as {filename}")
        
        image_count += 1

def listener():
    rospy.init_node('realsense_image_saver', anonymous=True)
    rospy.Subscriber("user_input", String, callback)
    while not rospy.is_shutdown():
        frame = Get_RGB_Frame()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("photo", frame)    
        cv2.waitKey(1) 



def Get_Depth_K():
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    K = np.zeros((3, 3))
    K[0][0] = depth_intrinsics.fx
    K[0][2] = depth_intrinsics.ppx
    K[1][1] = depth_intrinsics.fy
    K[1][2] = depth_intrinsics.ppy
    K[2][2] = 1
    return K

def Get_Color_K():
    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    K = np.zeros((3, 3))
    K[0][0] = color_intrinsics.fx
    K[0][2] = color_intrinsics.ppx
    K[1][1] = color_intrinsics.fy
    K[1][2] = color_intrinsics.ppy
    K[2][2] = 1
    return K


def Get_PointCloud(sample_length = 10, 
                   is_decimation_filter = False, 
                   is_spatial_filter = False, 
                   is_temporal_filter = False, 
                   is_hole_filling_filter = False,
                   is_depth_to_disparity = False,
                   is_disparity_to_depth = False):
    #output realsense point cloud as a np array(nx3)
    #
    #Effectively reduces the depth scene complexity
    decimation = rs.decimation_filter()
    #1D edge-preserving spatial filter using high-order domain transform
    spatial = rs.spatial_filter()
    #The temporal filter is intended to improve the depth data persistency by manipulating per-pixel values based on previous frames
    temporal = rs.temporal_filter()
    #The filter implements several methods to rectify missing data in the resulting image
    hole_filling = rs.hole_filling_filter()

    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)


    frames_list = []
    for f in range(sample_length):
        frame = pipeline.wait_for_frames()
        frames_list.append(frame.get_depth_frame())
    
    processed_frame = frames_list[0]
    for f in frames_list:
        if is_decimation_filter:
            processed_frame = decimation.process(f)
        if is_spatial_filter:
            processed_frame = spatial.process(f)
        if is_temporal_filter:
            processed_frame = temporal.process(f)
        if is_hole_filling_filter:
            processed_frame = hole_filling.process(f)
        if is_depth_to_disparity:
            processed_frame = depth_to_disparity.process(f)
        if is_disparity_to_depth:
            processed_frame = disparity_to_depth.process(f)

    points = pc.calculate(processed_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    return verts

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()