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
import sensor_msgs.point_cloud2 as pcl2
import math
import sys
sys.path.append('/home/chen/catkin_ws/src')  # 將 src 加入模組路徑
from node_control.script.listen_python_move_to_fixture import new_monitor,move_send_script


class_names_dict={0: 'aluextru', 1: 'bin', 2: 'twpipe'}
class_colors = {
    0: (255, 0, 0),   # 類別 0 - 紅色
    1: (0, 255, 0),   # 類別 1 - 綠色
    2: (0, 0, 255),   # 類別 2 - 藍色
    # 添加更多類別顏色
}

image_path_test='/home/chen/圖片'
weight_path = r"Segmentation_Train/results/training_results5/weights/best.pt"
model = YOLO(weight_path)

# exit()
# print(dir(rs))
# help(rs.rs2_deproject_pixel_to_point)
class view_point_cloud:
    def __init__(self,view_id,color_image=None,depth_image=None,intrinsics=None):
        self.intrinsics=intrinsics
        self.color_image=color_image
        self.depth_image=depth_image
        self.mark_image,self.mask_pixel_list,self.class_id_list,self.confidence_score_list=self.get_mask_data_accurate(color_image)

        

        self.all_instance_points,self.all_instance_original_colors,self.all_instance_draw_colors=self.merge_mask_points()
    
        self.o3d_pcd=self.generate_o3d_pointcloud()
        
        self.view_id=view_id

    
    def extract_pose_values(self,coord_string):
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


    def ask_flange_pose(self):
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
                pose_values = self.extract_pose_values(coord_string)
                if pose_values:
                    pose_values = np.asarray(pose_values)
                    # robot_pose=pose_values
                    return pose_values
        except rospy.ServiceException as e:
                    rospy.logerr("Error AskItem to robot: {}".format(e))


    def pose_to_matrix(self,pose, degrees=True,milimeter=True):
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


    def matrix_to_pose(self,matrix, degrees=True,milimeter=True):
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


    def get_mask_data(self,image_src):
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
        self.mark_image=original_image
        self.mask_pixel_list=mask_pixel_list
        self.class_id_list=class_id_list
        self.confidence_score_list=confidence_score_list
                    # break
        return original_image,mask_pixel_list,class_id_list,confidence_score_list

    def get_mask_data_accurate(self,image_src):
        image=image_src.copy()
        results = model.predict(
            source=image,
            verbose=False,
            save=False,
            retina_masks=True,  # ← ✅ 確保輸出高解析度遮罩
            device='cpu'
        )

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


    def pixel_to_3d_points(self,mask_pixel_np, depth_image, intrinsics ):
        """
        將 mask 的 N x 2 pixel 座標轉為 3D 空間座標（N x 3）

        mask_pixel_np: (N, 2)，每列是 [x, y]
        depth_image: 對齊 color 的深度圖（單位 mm）
        intrinsics: RealSense 相機內參（對齊 color 用 color stream 的內參）
        """
        points_3d = []
        pose = self.ask_flange_pose()
        T_flange2base = self.pose_to_matrix(pose)
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
        return points_3d_in_base # shape (M, 3)


    def merge_mask_points(self):
        """
        將 mask 的 N x 2 pixel 座標轉為 3D 空間座標（N x 3）

        mask_pixel_np: (N, 2)，每列是 [x, y]
        depth_image: 對齊 color 的深度圖（單位 mm）
        intrinsics: RealSense 相機內參（對齊 color 用 color stream 的內參）
        """
        all_instance_points = []
        all_instance_original_colors = []  # 新增一個列表來存儲每個實例的顏色
        all_instance_draw_colors = []  # 新增一個列表來著色

        for i, pix2d in enumerate(self.mask_pixel_list):
            valid_pix2d = pix2d[self.depth_image[pix2d[:, 1], pix2d[:, 0]] > 0]  # 過濾掉深度值為 0 的像素
            # print(valid_pix2d.shape)
            pts3d = self.pixel_to_3d_points(valid_pix2d, self.depth_image, self.intrinsics)
            all_instance_points.append(pts3d)


            # 根據類別名稱查找顏色
            class_name = self.class_id_list[i]
            class_index = next((k for k, v in class_names_dict.items() if v == class_name), None)
            color = class_colors.get(class_index, (255, 255, 255))  # 默認白色
            normalized_color = [c / 255.0 for c in color]
            all_instance_draw_colors.extend([normalized_color] * len(pts3d))

            # 提取有效像素的原始顏色
            original_colors = color_image[valid_pix2d[:, 1], valid_pix2d[:, 0]]
            normalized_colors = original_colors / 255.0
            all_instance_original_colors.extend(normalized_colors)
            print(np.array(all_instance_draw_colors).shape,np.array(all_instance_original_colors).shape)
        self.all_instance_points = all_instance_points
        self.all_instance_original_colors = all_instance_original_colors
        self.all_instance_draw_colors = all_instance_draw_colors
        return all_instance_points, all_instance_original_colors, all_instance_draw_colors
    
    def generate_o3d_pointcloud(self):
        np_all_instance_points = np.vstack(self.all_instance_points)
        np_all_instance_original_colors = np.array(self.all_instance_original_colors)[:,::-1]
        pcd=o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_all_instance_points)
        # pcd.colors = o3d.utility.Vector3dVector(np.array(all_instance_draw_colors))  # 設定點雲顏色
        pcd.colors = o3d.utility.Vector3dVector(np_all_instance_original_colors)
        return pcd      


def preprocess_point_cloud(pcd, voxel_size=0.001)->o3d.geometry.PointCloud:
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    return pcd_down

def icp_align_origin(source, target, threshold=0.02):
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return result.transformation

def icp_align_robust_plane(source, target, threshold=0.02, sigma=0.05):
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.identity(4), estimator
    )
    return result.transformation, result

def o3d_to_ros_pointcloud(pcd, frame_id="map"):
    # 將 Open3D 的點雲轉為 PointCloud2 ROS 訊息
    points = np.asarray(pcd.points)
    colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
    data = []

    for i in range(len(points)):
        x, y, z = points[i]
        r, g, b = colors[i]
        rgb = int(r) << 16 | int(g) << 8 | int(b)
        data.append([x, y, z, rgb])

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
    ]

    pc2_msg = pcl2.create_cloud(header, fields, data)
    return pc2_msg

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

def roi_cut(color_image,depth_image, pt1:tuple=(0,0),pt2:tuple=(1280,720)):
    x1, y1 = pt1
    x2, y2 = pt2

    # === 切割 ROI ===
    color_image = color_image[y1:y2, x1:x2]
    depth_image = depth_image[y1:y2, x1:x2]
    return color_image, depth_image



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

if __name__ == "__main__":
    # Create a pipeline
    pipeline = rs.pipeline()
    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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

    align_to = rs.stream.color
    align = rs.align(align_to)

    # 初始化 Hole Filling Filter
    # hole_filling = rs.hole_filling_filter()
    num_views=2
    # Streaming loop
    view_point_cloud_list=[]
    try:
        for i in range(num_views):
            input(f"\n📸 移動到第 {i+1} 視角後按 Enter 擷取...")
            for _ in range(10): frames = pipeline.wait_for_frames()  # 等幾幀讓資料穩定
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # 應用 Hole Filling Filter
            filtered_depth_frame = post_process(aligned_depth_frame)

            # 將深度影像轉換為 NumPy 陣列
            depth_image = np.asanyarray(filtered_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            cv2.imwrite(os.path.join(image_path_test, f'{i}_detect.png'), color_image)
            # exit()
            view_point_cloud_list.append(
                view_point_cloud(i,
                                 color_image,
                                 depth_image,
                                 profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()))
            
                    # 預處理點雲
        processed_point_clouds = [preprocess_point_cloud(vpc.o3d_pcd) for vpc in view_point_cloud_list]
        
        # 設定基準點雲
        target = processed_point_clouds[0]
        combined_pcd = target  # 初始化合併點雲
        
        # 疊合其他點雲到基準點雲
        for i in range(1, len(processed_point_clouds)):
            source = processed_point_clouds[i]
            print(f"正在進行第 {i} 個點雲的 ICP 疊合...")
            
            # 使用 icp_align_robust_plane 進行疊合
            transformation, result = icp_align_robust_plane(source, target, threshold=0.02, sigma=0.05)
            print(f"ICP 疊合完成，轉換矩陣：\n{transformation}")
            
            # 將轉換矩陣應用到點雲
            source.transform(transformation)
            
            # 合併點雲
            combined_pcd += source
        
        
        
    
                
                
        # 建立座標系
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        
        
        # 設定視角參數
        zoom = 2
        
        front = [-0.5, 0.0, 0.0]
        lookat = [0.0, 0.0, 0.0]
        up = [0.0, 0.0, 1.0]

        # msg=o3d_to_ros_pointcloud(pcd_combined)
        
        # 顯示點雲和座標系
        geometries_to_draw = [combined_pcd] + [axis]
        o3d.visualization.draw_geometries(geometries_to_draw,
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


