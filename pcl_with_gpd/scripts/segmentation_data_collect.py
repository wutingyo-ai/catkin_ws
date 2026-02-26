#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import glob

# 設定儲存路徑
left_dir = '/home/chen/segmentation/left'
right_dir = '/home/chen/segmentation/right'

# 確保資料夾存在
os.makedirs(left_dir, exist_ok=True)
os.makedirs(right_dir, exist_ok=True)

# 初始化 RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
pipeline.start(config)

# 設定對齊物件
align = rs.align(rs.stream.color)

print("請按下數字鍵 1（儲存至 left）或 2（儲存至 right），按下 q 鍵退出。")

try:
    while True:
        # 等待新的影格
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if not color_frame:
            continue

        # 將影像轉換為 numpy 陣列
        color_image = np.asanyarray(color_frame.get_data())

        # 顯示影像
        cv2.imshow('RealSense RGB', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('1'):
            # 計算目前 left 資料夾中的影像數量
            left_count = len(glob.glob(os.path.join(left_dir, 'left_*.jpg')))
            filename = os.path.join(left_dir, f'left_{left_count}.jpg')
            cv2.imwrite(filename, cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            print(f'已儲存：{filename}')
        elif key == ord('2'):
            # 計算目前 right 資料夾中的影像數量
            right_count = len(glob.glob(os.path.join(right_dir, 'right_*.jpg')))
            filename = os.path.join(right_dir, f'right_{right_count}.jpg')
            cv2.imwrite(filename, cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            print(f'已儲存：{filename}')
        elif key == ord('q'):
            print("退出程式。")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
