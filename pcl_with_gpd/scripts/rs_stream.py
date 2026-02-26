#!/usr/bin/env python3.8
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import math
import time

def stream_color_depth_frame():
    # 1. 初始化 RealSense Pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # 2. 設定相機解析度與幀率 (RGB: 640x480, Depth: 640x480, 30 FPS)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # 3. 啟動相機
    pipeline.start(config)

    try:
        while True:
            # 4. 取得幀資料
            frames = pipeline.wait_for_frames()
            
            # 5. 取得深度與 RGB 影像
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            # 6. 轉換為 NumPy 陣列
            depth_image = np.asanyarray(depth_frame.get_data())  # 深度影像
            color_image = np.asanyarray(color_frame.get_data())  # RGB 影像

            # 7. 將深度影像轉換為 8-bit 顯示格式
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 8. 顯示 RGB 與深度影像
            cv2.imshow('RealSense RGB', color_image)
            cv2.imshow('RealSense Depth', depth_colormap)

            # 9. 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 10. 停止相機
        pipeline.stop()
        cv2.destroyAllWindows()

def stream_point_cloud():
    # 初始化 RealSense
    pipeline = rs.pipeline()
    config = rs.config()

    # 設定 RGB + 深度影像串流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 啟動相機
    pipeline.start(config)
    # 創建 Open3D 視覺化物件
    vis = o3d.visualization.Visualizer()
    vis.create_window("RealSense Point Cloud", width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # 對齊深度到 RGB
    align = rs.align(rs.stream.color)

    try:
        while True:
            # 取得影像
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                print("未獲取到深度或顏色影像")
                continue

            # 轉換 NumPy 陣列
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 檢查深度影像的最小值和最大值
            print(f"深度影像最小值: {np.min(depth_image)}, 最大值: {np.max(depth_image)}")

            # 取得相機內部參數
            intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

            # 產生點雲
            points = []
            colors = []
            height, width = depth_image.shape

            for y in range(0, height, 2):  # 降低解析度提升效能
                for x in range(0, width, 2):
                    depth = depth_image[y, x] * 0.001  # 轉換為公尺
                    if depth > 0 and depth < 5:  # 確保深度在合理範圍內
                        xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                        points.append(xyz)
                        colors.append([0.0, 0.0, 0.0])  # 將顏色設為黑色

            # 確保點雲不為空
            if len(points) > 0:
                pcd.points = o3d.utility.Vector3dVector(np.array(points))
                pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
                vis.update_geometry(pcd)  # 更新點雲
            else:
                print("生成的點雲為空")

            vis.poll_events()
            vis.update_renderer()

    finally:
        pipeline.stop()
        vis.destroy_window()


def main():
    stream_color_depth_frame()

if __name__=="__main__":
    main()


