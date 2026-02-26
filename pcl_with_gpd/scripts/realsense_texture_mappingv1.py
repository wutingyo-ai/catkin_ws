#!/usr/bin/env python3.8
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 10 #3 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 255
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        print(images.shape)
        
        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        # 假設您要獲取的像素座標
        pixel_x = 320  # X座標
        pixel_y = 240  # Y座標

        # 獲取該像素的深度值
        depth_value = depth_image[pixel_y, pixel_x] * depth_scale  # 轉換為公尺

        if depth_value > 0:  # 確保深度值有效
            # 獲取相機內部參數
            intrinsics = aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics()
            
            # 將 2D 像素座標轉換為 3D 空間座標
            xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel_x, pixel_y], depth_value)
            print(f"3D Point at pixel ({pixel_x}, {pixel_y}): {xyz}")
        else:
            print(f"該像素的深度值無效{depth_value}")

finally:
    pipeline.stop()

""" import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# 初始化 RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# 啟動相機，開啟 RGB + 深度流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 對齊深度影像到 RGB 影像
align = rs.align(rs.stream.color)

try:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        raise RuntimeError("未獲取影像")

    # 轉換影像為 NumPy 陣列
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # 獲取相機內部參數
    intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

    # 生成點雲
    point_cloud = []
    colors = []
    height, width = depth_image.shape

    for y in range(height):
        for x in range(width):
            depth = depth_image[y, x] * 0.001  # 轉換為公尺
            if depth > 0:
                # 直接使用 RealSense 提供的函式轉換為 3D 座標
                xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                point_cloud.append(xyz)
                colors.append(color_image[y, x] / 255.0)  # 標準化 RGB 顏色

    # 建立 Open3D 點雲物件
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(point_cloud))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    # 顯示點雲
    o3d.visualization.draw_geometries([pcd])

finally:
    pipeline.stop()
 """
