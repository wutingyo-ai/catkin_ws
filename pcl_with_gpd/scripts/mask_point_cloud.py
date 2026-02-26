
# ------------------------------------------------------
#!/usr/bin/env python3.8
#####################################################
##              Align Depth to Color               ##
#####################################################
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from ultralytics import YOLO
import os
import rospy
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import AskItem, AskItemRequest
from scipy.spatial.transform import Rotation as R_scipy
import re
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import math
import sys
sys.path.append('/home/chen/catkin_ws/src')  # 將 src 加入模組路徑
from node_control.script.listen_python_move_to_fixture import new_monitor,move_send_script


# exit()
# print(dir(rs))
# help(rs.rs2_deproject_pixel_to_point)





class_names_dict={0: 'aluextru', 1: 'bin', 2: 'twpipe'}
class_colors = {
    0: (255, 0, 0),   # 類別 0 - 紅色
    1: (0, 255, 0),   # 類別 1 - 綠色
    2: (0, 0, 255),   # 類別 2 - 藍色
    # 添加更多類別顏色
}

image_path_test='/home/chen/圖片/detect.png'
weight_path = r"Segmentation_Train/results/training_results5/weights/best.pt"
model = YOLO(weight_path)
# exit()



def pose_to_matrix(pose, degrees=True,milimeter=True):
    """
    將位置 (x, y, z) 和歐拉角 (rx, ry, rz) 轉換為 4x4 的齊次轉換矩陣。
    
    參數:
        x, y, z: 平移量
        rx, ry, rz: 歐拉角（單位為弧度或度）
        degrees: 如果為 True，則輸入的角度為度；否則為弧度。
    
    返回:
        4x4 的齊次轉換矩陣（numpy.ndarray）
        
    """
    x,y,z,rx,ry,rz=pose
    if degrees:
        rotation = R_scipy.from_euler('xyz', [rx, ry, rz], degrees=True)
    else:
        rotation = R_scipy.from_euler('xyz', [rx, ry, rz], degrees=False)
    matrix = np.eye(4)
    
    matrix[:3, :3] = rotation.as_matrix() 
    
    if milimeter:
        
        matrix[:3, 3] = [x/1000, y/1000, z/1000]
    else:
        matrix[:3, 3] = [x, y, z]
    return matrix

def matrix_to_pose(matrix, degrees=True,milimeter=True):
    """
    將 4x4 的齊次轉換矩陣還原為位置和歐拉角。
    
    參數:
        matrix: 4x4 的齊次轉換矩陣（numpy.ndarray）
        degrees: 如果為 True，則輸出的角度為度；否則為弧度。
    
    返回:
        (x, y, z, rx, ry, rz): 位置和平移量
    """
    if degrees:
        rotation = R_scipy.from_matrix(matrix[:3, :3])
        rx, ry, rz = rotation.as_euler('xyz', degrees=True)
    else:
        rotation = R_scipy.from_matrix(matrix[:3, :3])
        rx, ry, rz = rotation.as_euler('xyz', degrees=False)

    if milimeter:
        x, y, z = matrix[:3, 3]/1000
    else:
        x,y,z=matrix[:3, 3]
    return x, y, z, rx, ry, rz

def extract_pose_values(coord_string):
    # 使用正則表達式提取花括號內的內容
    match = re.search(r'\{(.*?)\}', coord_string)
    if match:
        # 將提取的內容按逗號分割
        values_str = match.group(1).split(',')
        # 將字符串轉換為浮點數
        values = [float(v.strip()) for v in values_str]
        return values
    else:
        return None

def ask_flange_pose():
    # global robot_pose
    client = rospy.ServiceProxy('tm_driver/ask_item', AskItem)
    request = AskItemRequest()
    request.id = "demo"
    request.wait_time = 0.5
    request.item = "Coord_Robot_Flange"
    try:
        response = client(request)
        if response.ok:
            rospy.loginfo("AskItem to robot: item is {}, value is {}\n".format(request.item, response.value))
            coord_string = response.value
            pose_values = extract_pose_values(coord_string)
            if pose_values:
                pose_values = np.asarray(pose_values)
                # robot_pose=pose_values
                return pose_values
    except rospy.ServiceException as e:
                rospy.logerr("Error AskItem to robot: {}".format(e))



# 遍歷資料夾中的所有圖片

def get_image_path_list(test_image_path = r"/home/chen/Segmentation_Train/train_data/test/images"):
    # 確保測試圖片存在
    if not os.path.exists(test_image_path):
        raise FileNotFoundError(f"測試圖片不存在: {test_image_path}")
    img_list=[]
    for image_name in os.listdir(test_image_path):
        # 確保是圖片檔案
        if image_name.lower().endswith(('.jpg', '.jpeg', '.png')):
            image_path = os.path.join(test_image_path, image_name)
            img_list.append(image_path)
            # print(f"正在處理圖片: {image_path}")
    return img_list

def get_mask_data(image_src):
    image=image_src.copy()
    results = model.predict(source=image,verbose=False)
    
    # 讀取原始圖片
    # original_image = cv2.imread(image)
    original_image = image
    mask_pixel_list=[]
    class_id_list=[]
    confidence_score_list=[]

    for result in results:
        masks = result.masks  # 獲取分割遮罩  
        classes = result.boxes.cls.cpu().numpy()  # 獲取類別索引  
        confidence=result.boxes.conf.cpu().numpy()  # 獲取置信度
        if masks is not None:
            for mask, cls,conf in zip(masks.data, classes,confidence):
                # 將遮罩轉換為二值圖像
                mask = mask.cpu().numpy().astype(np.uint8) * 255
                # print(conf)
                confidence_score_list.append(conf)
                
                # 獲取對應類別的顏色
                color = class_colors.get(int(cls), (255, 255, 255))  # 默認白色
                class_name = class_names_dict.get(int(cls), "Unknown")
                class_id_list.append(class_name)

                # 獲取遮罩內部所有像素的座標
                y_coords, x_coords = np.where(mask > 0)  # 獲取非零像素的 y 和 x 座標
                pixel_coords = np.array([x_coords, y_coords])  # 2×N 矩陣
                mask_pixel_list.append(pixel_coords.T)

                """ 
                # 測試遮罩座標是否正確
                for x,y in pixel_coords.T:
                    original_image[y,x]=(0,0,0)
                print(pixel_coords[:,0])
                """

                # exit()

                # 將遮罩疊加到原始圖片
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(original_image, contours, -1, color, thickness=1)
                
                # 創建一個與原始圖片相同大小的透明遮罩
                overlay = original_image.copy()
                alpha = 0.5  # 設定透明度 (0.0 完全透明, 1.0 完全不透明)

                # 將遮罩應用顏色
                overlay[mask > 0] = color

                # 將透明遮罩疊加到原始圖片
                cv2.addWeighted(overlay, alpha, original_image, 1 - alpha, 0, original_image)
                    # 計算遮罩的中心點
                for contour in contours:
                    M = cv2.moments(contour)
                    if M["m00"] > 0:  # 防止除以零
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        # 在中心點添加類別文字標籤
                        cv2.putText(original_image, class_name, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.6, color, 2, cv2.LINE_AA)
                print(class_name,conf,pixel_coords.T.shape)
                # break
    return original_image,mask_pixel_list,class_id_list,confidence_score_list

def get_mask_data_accurate(image_src):
    def roi_cut(color_image, pt1:tuple=(0,0),pt2:tuple=(1280,720)):
        x1, y1 = pt1
        x2, y2 = pt2

        # === 切割 ROI ===
        color_image = color_image[y1:y2, x1:x2]
        
        return color_image,pt1

    image=image_src.copy()
    image,pt1=roi_cut(image,pt1=(230,10),pt2=(1040,700))
    
    results = model.predict(
        source=image,
        verbose=False,
        save=False,
        retina_masks=True,  # ← ✅ 確保輸出高解析度遮罩
        device='cpu'
    )
    x1,y1=pt1
    # image = cv2.imread(image_path)
    h, w = image.shape[:2]

    mask_pixel_list = []
    class_id_list = []
    confidence_score_list = []

    for result in results:
        masks = result.masks.data  # [n, h, w] torch.Tensor
        classes = result.boxes.cls.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()

        for i, (cls, conf) in enumerate(zip(classes, scores)):
            class_name = class_names_dict.get(int(cls), "Unknown")
            class_id_list.append(class_name)
            confidence_score_list.append(conf)

            # 提取單一 mask
            mask_tensor = masks[i]  # shape: (h, w)
            mask = mask_tensor.cpu().numpy().astype(np.uint8) * 255
            mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

            # 獲取遮罩像素點位置
            y, x = np.where(mask_resized > 0)
            y=y+y1
            x=x+x1
            pixel_coords = np.stack([x, y], axis=1)
            mask_pixel_list.append(pixel_coords)

            # 遮罩上色與疊加
            color = class_colors.get(int(cls), (255, 255, 255))
            overlay = image.copy()
            contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, thickness=cv2.FILLED)
            image = cv2.addWeighted(overlay, 0.5, image, 0.5, 0)

            # 邊框與文字
            cv2.drawContours(image, contours, -1, color, thickness=2)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    label = f"{class_name} {conf:.2f}"
                    cv2.putText(image, label, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return image, mask_pixel_list, class_id_list, confidence_score_list    

def pixel_to_3d_points(mask_pixel_np, depth_image, intrinsics ):
    """
    將 mask 的 N x 2 pixel 座標轉為 3D 空間座標（N x 3）

    mask_pixel_np: (N, 2)，每列是 [x, y]
    depth_image: 對齊 color 的深度圖（單位 mm）
    intrinsics: RealSense 相機內參（對齊 color 用 color stream 的內參）
    """
    points_3d = []
    pose = ask_flange_pose()
    T_flange2base = pose_to_matrix(pose)
    T_cam2gripper=np.array(
    [[-0.995,0.099,-0.017,0.038],
    [-0.098,-0.992,-0.079,-0.075],
    [-0.025,-0.076,0.997,0.038],
    [0.000,0.000,0.000,1.000]])

    for x, y in mask_pixel_np:
        depth = depth_image[y, x] * 0.001  # mm → m
        if depth == 0: continue  # 忽略無效點
        pt3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)  # 回傳 [X, Y, Z]
        points_3d.append(pt3d)
    
    
    # 將 points_3d 轉換為 4 x M 的齊次矩陣形式
    points_3d = np.array(points_3d)  # 原始 M x 3
    points_3d_homogeneous = np.hstack((points_3d, np.ones((points_3d.shape[0], 1)))).T  # 轉換為 4 x M
    points_3d_in_base= (T_flange2base @ T_cam2gripper @ points_3d_homogeneous).T[:, :3]  # 轉換為 base 座標系
    
    return points_3d_in_base  # shape (M, 3)

def post_process_0(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()
    # === Apply filters ===
    filtered = decimation.process(depth_frame)
    filtered = spatial.process(filtered)
    filtered = temporal.process(filtered)
    filtered = hole_filling.process(filtered)
    return filtered

def post_process(depth_frame):
    # === Disparity transformation ===
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    depth_frame = depth_to_disparity.process(depth_frame)

    # === Decimation Filter ===
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 1)
    depth_frame = decimation.process(depth_frame)

    # === Spatial Filter ===
    spatial = rs.spatial_filter(
        smooth_alpha=0.25, 
        smooth_delta=50.0, 
        magnitude=3.0,
        hole_fill=2.0
    )
    depth_frame = spatial.process(depth_frame)

    # === Temporal Filter ===
    temporal = rs.temporal_filter(0.25, 100.0, 8)  # alpha, delta, persistency_mode
    depth_frame = temporal.process(depth_frame)

    # === Convert back to depth ===
    depth_frame = disparity_to_depth.process(depth_frame)

    # === Hole Filling ===
    hole_filling = rs.hole_filling_filter()
    hole_filling.set_option(rs.option.holes_fill, 1)
    depth_frame = hole_filling.process(depth_frame)

    return depth_frame




# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# Start streaming
profile = pipeline.start(config)
device = profile.get_device()

# === Apply JSON Advanced Settings ===
try:
    advanced_mode = rs.rs400_advanced_mode(device)
    if not advanced_mode.is_enabled():
        print("🛠 正在啟用 advanced mode ...")
        advanced_mode.toggle_advanced_mode(True)
        import time
        time.sleep(1)
    # 套用 JSON
    with open('/home/chen/realsense_setting/realsesnse_setting3.json', 'r') as file:
        json_text = file.read()
        advanced_mode.load_json(json_text)
    print("✅ JSON 設定已成功載入")
except Exception as e:
    print("⚠️ 裝置不支援 advanced_mode，或初始化失敗：", e)



#####################################################
# # 取得深度感測器
# depth_sensor = profile.get_device().first_depth_sensor()
# # 關閉自動曝光
# depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
# # 設定曝光值為 715
# depth_sensor.set_option(rs.option.exposure, 800)
# print("Auto Exposure disabled and exposure set to 800.")
#####################################################



align_to = rs.stream.color
align = rs.align(align_to)

# 初始化 Hole Filling Filter
# hole_filling = rs.hole_filling_filter()

# Streaming loop
try:
    for _ in range(30): frames = pipeline.wait_for_frames()  # 等幾幀讓資料穩定

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # 應用 Hole Filling Filter
    # filtered_depth_frame = hole_filling.process(aligned_depth_frame)
    filtered_depth_frame = post_process(aligned_depth_frame)

    # 將深度影像轉換為 NumPy 陣列
    depth_image = np.asanyarray(filtered_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # color_image,pt1=roi_cut(color_image,pt1=(230,10),pt2=(1040,700))
    cv2.imwrite(image_path_test, color_image)
    print(depth_image.shape)
    # exit()
#####################################################
    # # Remove background - Set pixels further than clipping_distance to grey
    # grey_color = 255
    # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # Render images:
    #   depth align to color on left
    #   depth on right
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # images = np.hstack((bg_removed, depth_colormap))
    # print(images.shape)
    
    # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    # cv2.imshow('Align Example', images)
    # key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    # if key & 0xFF == ord('q') or key == 27:
    #     cv2.destroyAllWindows()
    #     break
#####################################################
    # # 假設您要獲取的像素座標
    # pixel_x = 320  # X座標
    # pixel_y = 240  # Y座標

    # # 獲取該像素的深度值
    # depth_value = depth_image[pixel_y, pixel_x] * depth_scale  # 轉換為公尺

    # if depth_value > 0:  # 確保深度值有效
    #     # 獲取相機內部參數
    #     intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        
    #     # 將 2D 像素座標轉換為 3D 空間座標
    #     xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel_x, pixel_y], depth_value)
    #     print(f"3D Point at pixel ({pixel_x}, {pixel_y}): {xyz}")
    # else:
    #     print(f"該像素的深度值無效{depth_value}")
#####################################################    
    # # Define ROI (example: a rectangle)
    # roi_start_x, roi_start_y = 0, 0  # ROI 起始點
    # roi_width, roi_height = 200, 200      # ROI 寬度和高度
    # roi_end_x = roi_start_x + roi_width
    # roi_end_y = roi_start_y + roi_height

    # # Draw ROI on the color image
    # cv2.rectangle(color_image, (roi_start_x, roi_start_y), (roi_end_x, roi_end_y), (0, 255, 0), 2)

    # # Extract ROI from depth and color images
    # roi_depth = depth_image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]
    # roi_color = color_image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]
#####################################################
    # point_cloud = []
    # colors = []
    
    mark_image,mask_pixel_list,class_id_list,confidence_score_list=get_mask_data_accurate(color_image)
    
    intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
    
    all_instance_points = []
    all_instance_original_colors = []  # 新增一個列表來存儲每個實例的顏色
    all_instance_draw_colors = []  # 新增一個列表來著色

    for i, pix2d in enumerate(mask_pixel_list):

        valid_pix2d = pix2d[depth_image[pix2d[:, 1], pix2d[:, 0]] > 0]  # 過濾掉深度值為 0 的像素
        
        # print(valid_pix2d.shape)
        pts3d = pixel_to_3d_points(valid_pix2d, depth_image, intrinsics)
        all_instance_points.append(pts3d)

        # 根據類別名稱查找顏色
        class_name = class_id_list[i]
        class_index = next((k for k, v in class_names_dict.items() if v == class_name), None)
        color = class_colors.get(class_index, (255, 255, 255))  # 默認白色
        normalized_color = [c / 255.0 for c in color]
        all_instance_draw_colors.extend([normalized_color] * len(pts3d))

        # 提取有效像素的原始顏色
        original_colors = color_image[valid_pix2d[:, 1], valid_pix2d[:, 0]]
        normalized_colors = original_colors / 255.0
        all_instance_original_colors.extend(normalized_colors)
        

        print(np.array(all_instance_draw_colors).shape,np.array(all_instance_original_colors).shape)
    
        

        

        

    # print(all_instance_points[0].shape)
    # for y in range(color_image.shape[0]):
    #     for x in range(color_image.shape[1]):
    #         depth = depth_image[y, x] * 0.001  # Convert to meters
    #         if depth > 0:
    #             xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [roi_start_x + x, roi_start_y + y], depth)
    #             point_cloud.append(xyz)
    #             # Convert BGR to RGB
    #             rgb_color = roi_color[y, x] / 255.0
    #             colors.append([rgb_color[2], rgb_color[1], rgb_color[0]])  # Switch to RGB

    # Create Open3D point cloud object
    # 建立 Open3D 點雲物件
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack(all_instance_points))
    # pcd.colors = o3d.utility.Vector3dVector(np.array(all_instance_draw_colors))  # 設定點雲顏色
    pcd.colors = o3d.utility.Vector3dVector(np.array(all_instance_original_colors)[:,::-1])  # 設定點雲顏色

        
        
    # 建立座標系
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    
    # 設定視角參數
    zoom = 2
    
    front = [-0.5, 0.0, 0.0]
    lookat = [0.0, 0.0, 0.0]
    up = [0.0, 0.0, 1.0]

    # msg=o3d_to_ros_pointcloud(pcd_combined)
    
    # 顯示點雲和座標系
    o3d.visualization.draw_geometries([pcd, axis],
                                    zoom=zoom,
                                    front=front,
                                    lookat=lookat,
                                    up=up,
                                    window_name="RGB PointCloud from RealSense")

    # Display the color image with ROI
    # cv2.imshow('Color Image with ROI', color_image)

    # # Display the point cloud
    # if input("go")=="go":
    #     o3d.visualization.draw_geometries([pcd])

    # key = cv2.waitKey(0)
    # if key & 0xFF == ord('q'):
    #     break
    # elif key & 0xff== ord('m'):
    #     o3d.visualization.draw_geometries([pcd])
    #     continue
        
        

finally:
    pipeline.stop()
    # cv2.destroyAllWindows()

# if __name__=="__main__":

