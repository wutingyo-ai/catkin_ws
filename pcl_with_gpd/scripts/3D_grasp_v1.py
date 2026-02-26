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
from visualization_msgs.msg import Marker,MarkerArray
import sensor_msgs.point_cloud2 as pcl2
from visualization_msgs.msg import MarkerArray  # 如果未來改用 MarkerArray
from std_msgs.msg import ColorRGBA    
import math
import sys
sys.path.append('/home/chen/catkin_ws/src')  # 將 src 加入模組路徑
from robot_control import *

# from node_control.script.listen_python_move_to_fixture import new_monitor,move_send_script
from gpd_ros.msg import GraspConfigList

class_names_dict={0: 'aluextru', 1: 'bin', 2: 'twpipe'}
class_colors = {
    0: (255, 0, 0),   # 類別 0 - 紅色
    1: (0, 255, 0),   # 類別 1 - 綠色
    2: (0, 0, 255),   # 類別 2 - 藍色
    # 添加更多類別顏色
}

image_path_test='/home/chen/catkin_ws/src/pcl_with_gpd/picture'
# weight_path = r"/home/chen/catkin_ws/src/pcl_with_gpd/weight/New_best.pt"
weight_path = r"/home/chen/catkin_ws/src/pcl_with_gpd/weight/2025_06_28_best.pt"
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


    '''def get_mask_data(self,image_src):
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
        return original_image,mask_pixel_list,class_id_list,confidence_score_list'''


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
        np_all_instance_draw_colors = np.array(self.all_instance_draw_colors)[:,:]
        pcd=o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_all_instance_points)
        pcd.colors = o3d.utility.Vector3dVector(np_all_instance_draw_colors)  # 設定點雲顏色
        # pcd.colors = o3d.utility.Vector3dVector(np_all_instance_original_colors)
        return pcd      

    def get_mask_data_accurate(self,image_src):
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
        
def preprocess_point_cloud(pcd, voxel_size=0.003)->o3d.geometry.PointCloud:
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

def icp_align_robust_plane(source, target, threshold=0.01, sigma=0.01):
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.identity(4), estimator
    )
    return result.transformation, result

def o3d_to_ros_pointcloud(pcd, frame_id="base"):
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
    # hole_filling = rs.hole_filling_filter()
    # hole_filling.set_option(rs.option.holes_fill, 1)
    # depth_frame = hole_filling.process(depth_frame)

    return depth_frame


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
  
def collision_detect_z(position, approach, binormal, axis, bbox_id,
                                   hand_depth=0.07, hand_height=0.02,
                                   outer_diameter=0.105, finger_width=0.01,
                                   table_z=0.08828, frame_id="base"):
    hw = 0.5 * outer_diameter - 0.5 * finger_width
    R = np.column_stack((approach, binormal, axis))
    z_compensation = 0.0
    z_compensation_mm=0.0
    left_center = position - hw * binormal + 0.5 * hand_depth * approach
    right_center = position + hw * binormal + 0.5 * hand_depth * approach
    left_tip = position - hw * binormal + hand_depth * approach
    right_tip = position + hw * binormal + hand_depth * approach

    # markers = MarkerArray()

    # # left & right box
    # for idx, center in enumerate([left_center, right_center]):
    #     marker = Marker()
    #     marker.header.frame_id = frame_id
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "bbox"
    #     marker.id = bbox_id * 10 + idx
    #     marker.type = Marker.CUBE
    #     marker.action = Marker.ADD
    #     marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center
    #     quat = R_scipy.from_matrix(R).as_quat()
    #     marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quat
    #     marker.scale.x, marker.scale.y, marker.scale.z = hand_depth, finger_width, hand_height
    #     marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.4
    #     marker.lifetime = rospy.Duration(30)
    #     markers.markers.append(marker)

    # fingertips
    # for idx, tip in enumerate([left_tip, right_tip]):
    #     marker = Marker()
    #     marker.header.frame_id = frame_id
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "fingertip"
    #     marker.id = bbox_id * 10 + 2 + idx
    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD
    #     marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = tip
    #     marker.scale.x = marker.scale.y = marker.scale.z = 0.01
    #     marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
    #     marker.lifetime = rospy.Duration(30)
    #     markers.markers.append(marker)

    # 碰撞檢查
    tip_z_min = min(left_tip[2], right_tip[2])
    is_danger = tip_z_min < table_z
    if is_danger:
            z_compensation = table_z+0.005- tip_z_min
            z_compensation_mm = z_compensation * 1000  # 轉換為 mm
            rospy.logwarn(f"⚠️ 指尖碰撞檢測：tip_z_min={tip_z_min} , 需要補償 {z_compensation_mm:.3f} mm")
            
    
    return z_compensation_mm, is_danger


def grasp_callback(msg,combined_pcd):
    # global T_grasps_in_base_list,T_grasps_in_base_list,T_tool_list  # 使用全域變數
    
    
    T_grasps_in_base_list = []
    T_tool_list=[]
    gripper_base_position_mm_in_base_list= []
    tcp_pos_mm_in_base_list = []
    class_in_base_list = []
    
    rospy.loginfo("收到 %d 個抓取姿態", len(msg.grasps))
    if len(msg.grasps) !=0:
        have_grasps=True
        for i, grasp in enumerate(msg.grasps):
            # 將抓取姿態的資訊轉換為 np.array
            gripper_base_position = np.array([grasp.position.x, grasp.position.y, grasp.position.z])
            approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
            binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
            axis = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
            sample = np.array([grasp.sample.x, grasp.sample.y, grasp.sample.z]) 
            
            # === 推算 TCP 世界座標 ===
            hand_depth = 0.07        # 7cm 夾爪深度
            deepen_offset = 0.005     # 若開啟 deepen_hand，這裡應設為 0.01
            weight=1 # 夾爪深度的權重，若有 TCP_frame_transform，則設為 1
            tcp_offset = hand_depth / weight - deepen_offset
            tcp_position = gripper_base_position + tcp_offset * approach
            tcp_position_long = gripper_base_position + (hand_depth ) * approach  # 長度方向的 TCP 位置
            
            # 計算包圍長方體的範圍
            box_length = np.linalg.norm(tcp_position_long - gripper_base_position+0.01)  # 長方體的長度
            box_width = 20 / 1000  # 長方體的寬度（0.02mm）
            box_height = 20 / 1000  # 長方體的高度（0.02mm）
            # 遍歷點雲，篩選出在包圍長方體內的點

            points_in_box = []
            colors_in_box = []
            for point, color in zip(combined_pcd.points, combined_pcd.colors):
                # 計算點到 position 的向量
                vector_to_point = np.array(point) - gripper_base_position

                # 計算在 approach, binormal, axis 上的投影
                proj_length = np.dot(vector_to_point, approach)
                proj_width = np.dot(vector_to_point, binormal)
                proj_height = np.dot(vector_to_point, axis)

                # 判斷點是否在包圍長方體內
                if (0 <= proj_length <= box_length and
                    -box_width / 2 <= proj_width <= box_width / 2 and
                    -box_height / 2 <= proj_height <= box_height / 2):
                    points_in_box.append(point)
                    colors_in_box.append(color)

            # 統計範圍內的主要顏色
            if colors_in_box:
                colors_in_box = np.array(colors_in_box)
                avg_color = np.mean(colors_in_box, axis=0)  # 計算平均顏色
                # print(f"抓取姿態 {i} 的主要顏色: {avg_color}")

                # 判斷物體類型
                for class_id, class_color in class_colors.items():
                    normalized_color = np.array(class_color) / 255.0
                    if np.allclose(avg_color, normalized_color, atol=0.1):  # 顏色相似判斷
                        # print(f"抓取姿態 {i} 的物體類型: {class_names_dict[class_id]}")
                        class_in_base_list.append(class_names_dict[class_id])
                        break
                class_in_base_list.append(None)    
                
            else:
                print(f"抓取姿態 {i} 的範圍內沒有點雲")
                class_in_base_list.append(None)

            # === 建立 grasp frame ===
            R_grasp = np.column_stack((approach, binormal, axis))

            # 建立轉移矩陣
            T_tcp = R_and_t_to_T(R_grasp, tcp_position) 
            
            T_grasp_in_base = T_tcp  # 這就是抓取姿態的 TCP

            T_grasps_in_base_list.append(T_grasp_in_base)

            # 若要分離出旋轉與平移部分（可選）
            # R_grasp_in_base,t_grasp_in_base = T_to_R_and_t(T_grasp_in_base)


            # === 將 TCP 再轉為工具座標系（若有 TCP_frame_transform） ===
            TCP_frame_transform = np.array([
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]
            ])
            TCP_frame_transform_inv = np.linalg.inv(TCP_frame_transform)
            T_tool = T_grasp_in_base @ TCP_frame_transform_inv
            T_tool_list.append(T_tool)

            # 分解姿態
            R_tool, t_tool = T_to_R_and_t(T_tool)
            euler_angles = R_scipy.from_matrix(R_tool).as_euler('xyz', degrees=True)
            rx, ry, rz = euler_angles

            # Z 軸方向翻轉補正（若 rz 為負）
            if rz < 0:
                R_flip = R_scipy.from_euler('z', 180, degrees=True).as_matrix()
                T_flip = np.eye(4)
                T_flip[:3, :3] = R_flip
                T_tool = T_tool @ T_flip
                R_tool, t_tool = T_to_R_and_t(T_tool)
                rx, ry, rz = R_scipy.from_matrix(R_tool).as_euler('xyz', degrees=True)

            # TCP 點與 sample 點以 mm 為單位輸出
            tcp_pos_mm = t_tool * 1000
            gripper_base_position_mm = gripper_base_position * 1000
            x0, y0, z0 = gripper_base_position_mm
            x, y, z = [float(v) for v in tcp_pos_mm]
            
            z_compensation_mm, is_danger= collision_detect_z(
                gripper_base_position, approach, binormal, axis, bbox_id=i,
                hand_depth=0.07, hand_height=0.02,
                outer_diameter=0.105, finger_width=0.01,
                table_z=0.08828, frame_id="base"
            )
        
            if is_danger:
                # rospy.logwarn(f"[Grasp {i}] ⚠️ 指尖可能碰撞桌面！\n"
                #               f"origin_gripper_base_position x={x0:.2f}, y={y0:.2f}, z={z0:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}\n"
                #               f"origin_TCP x={x:.2f}, y={y:.2f}, z={z:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}\n"
                #               )
                z0 += z_compensation_mm
                z  += z_compensation_mm
            # 儲存結果
            gripper_base_position_mm_in_base_list.append([x0, y0, z0, rx, ry, rz])
            tcp_pos_mm_in_base_list.append([x, y, z, rx, ry, rz])
            # 輸出顯示
        #     color_print(f'[gripper_base_position ] x={x0:.2f}, y={y0:.2f}, z={z0:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}', color='cyan')
        #     color_print(f'[TCP] x={x:.2f}, y={y:.2f}, z={z:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}', color='cyan')

        color_print(f'Best_gripper_base_position = {gripper_base_position_mm_in_base_list[0]}', color='green')
        color_print(f'Best_TCP_position = {tcp_pos_mm_in_base_list[0]}', color='green')
        print('len =', len(T_grasps_in_base_list))

    else:
        rospy.logwarn("沒有收到任何抓取姿態，請檢查輸入訊息")
        have_grasps = False
        return [], [], [],have_grasps 
    return gripper_base_position_mm_in_base_list, tcp_pos_mm_in_base_list, class_in_base_list,have_grasps


def color_str(text, color="white"):
    color_codes = {
        "black":   "\033[30m",
        "red":     "\033[31m",
        "green":   "\033[32m",
        "yellow":  "\033[33m",
        "blue":    "\033[34m",
        "magenta": "\033[35m",
        "cyan":    "\033[36m",
        "white":   "\033[37m"
    }
    reset_code = "\033[0m"

    color = color.lower()
    if color not in color_codes:
        print(f"\033[31m[錯誤] 不支援的顏色：{color}\033[0m")
        return f"{color_codes[color]}{text}{reset_code}"

    

def color_print(text, color="white"):
    color_codes = {
        "black":   "\033[30m",
        "red":     "\033[31m",
        "green":   "\033[32m",
        "yellow":  "\033[33m",
        "blue":    "\033[34m",
        "magenta": "\033[35m",
        "cyan":    "\033[36m",
        "white":   "\033[37m"
    }
    reset_code = "\033[0m"

    color = color.lower()
    if color not in color_codes:
        print(f"\033[31m[錯誤] 不支援的顏色：{color}\033[0m")
        return

    print(f"{color_codes[color]}{text}{reset_code}")




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
    rospy.init_node('mask_point_cloud', anonymous=True)
    cloud_publisher=rospy.Publisher('cloud_stitched', PointCloud2, queue_size=10)

    gripper_pick(pick_distance=0,inital_bool=False)
    change_tcp("ChangeTCP(\"robotiq_origin_gripper\")")
    

    VIEW_POSE_1=[464.74 , -199.48 , 381.93 , 146.32 , 4.39 , 165.82] #TCP
    VIEW_POSE_2=[465.60 , 56.86 , 409.25 , 145.48 , 2.77 , 1.23] #TCP
    VIEW_POSE_3=[355.23 , -88.64 , 364.20 , 144.28 , -0.96 , 94.84] #TCP
    VIEW_POSE_LIST=[VIEW_POSE_1,VIEW_POSE_2,VIEW_POSE_3]
    PREPARE_POSE=[552.75 , -299.90 , 332.59 , 146.32 , 4.39 , 90.84] #TCP
    PUT_POSE_0=[655.84 , 499.26 , 239.60 , -180.00 , 0.00 , 180] #TCP
    PUT_POSE_1=[459.60 , 499.26 , 239.60 , -180.00 , 0.00 , 180] #TCP
    PUT_POSE_2=[252.89 , 499.26 , 239.60 , -180.00 , 0.00 , 180] #TCP
    RECYCLE_POSE=[-35.03 , 776.64 , 233.22, 180.00 ,0.00 ,-180.00]#TCP
    
    PUT_DOWN_POSE_0=[655.84 , 499.26 , 207.27 , -180.00 , 0.00 , 180] #TCP
    PUT_DOWN_POSE_1=[459.60 , 499.26 , 207.27 , -180.00 , 0.00 , 180] #TCP
    PUT_DOWN_POSE_2=[252.89 , 499.26 , 207.27 , -180.00 , 0.00 , 180] #TCP
    PUT_DOWN_RECYCLE_POSE=[-35.03 , 776.64 , 183.32, 180.00 ,0.00 ,-180.00]

    # Create a pipeline
    pipeline = rs.pipeline()
    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    CONFIG = rs.config()
    CONFIG.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    CONFIG.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # Start streaming
    profile = pipeline.start(CONFIG)
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

    while not rospy.is_shutdown():
    # Streaming loop
        view_point_cloud_list=[]
        move_script_with_monitor(VIEW_POSE_3,motion_type='PTP')
        for i, view_pose in enumerate(VIEW_POSE_LIST):
            # input(f"\n📸 移動到第 {i+1} 視角後按 Enter 擷取...")
            move_script_with_monitor(view_pose,motion_type='PTP')
            
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
        # processed_point_clouds = [preprocess_point_cloud(vpc.o3d_pcd) for vpc in view_point_cloud_list]
        processed_point_clouds = [vpc.o3d_pcd for vpc in view_point_cloud_list]
        
        # 設定基準點雲
        target = processed_point_clouds[0]
        combined_pcd = target  # 初始化合併點雲
        
        # 疊合其他點雲到基準點雲
        for i in range(1, len(processed_point_clouds)):
            source = processed_point_clouds[i]
            print(f"正在進行第 {i} 個點雲的 ICP 疊合...")
            
            # # 使用 icp_align_robust_plane 進行疊合
            # transformation, result = icp_align_robust_plane(source, target, threshold=0.02, sigma=0.05)
            # print(f"ICP 疊合完成，轉換矩陣：\n{transformation}")
            
            # # 將轉換矩陣應用到點雲
            # source.transform(transformation)
            
            # 合併點雲
            combined_pcd += source
        
        points = np.asarray(combined_pcd.points)
        # 設定 Z 值下限
        z_threshold = 0.07
        # 建立布林遮罩，只保留 Z 值 >= z_threshold 的點
        mask = points[:, 2] >= z_threshold
        # 套用遮罩並建立新的點雲物件
        filtered_pcd = combined_pcd.select_by_index(np.where(mask)[0])
        
        # combined_pcd = filtered_pcd   
        # cl, ind = combined_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=2.0)
        
        # combined_pcd=combined_pcd.select_by_index(ind)  
        combined_pcd=filtered_pcd
            
                
        # 建立座標系
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        
        
        # 設定視角參數
        zoom = 2
        
        front = [-0.5, 0.0, 0.0]
        lookat = [0.0, 0.0, 0.0]
        up = [0.0, 0.0, 1.0]

        # msg=o3d_to_ros_pointcloud(pcd_combined)
        o3d.io.write_point_cloud("/home/chen/catkin_ws/src/pcl_with_gpd/scripts/test.ply", combined_pcd, write_ascii=True)
        cloud_publisher.publish(o3d_to_ros_pointcloud(combined_pcd))
        # 顯示點雲和座標系
        geometries_to_draw = [combined_pcd] + [axis]
        
        '''o3d.visualization.draw_geometries(geometries_to_draw,
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
                                        '''
        voxelize_pcd=combined_pcd.voxel_down_sample(voxel_size=0.005)  # 將點雲進行體素化處理
        # bbox_array_pub = rospy.Publisher("/grasp_bbox_array", MarkerArray, queue_size=1)
            
        print("等待姿態消息...")
        have_grasps=False
        
        while  not rospy.is_shutdown():
            # color_print("沒有抓取姿態，請檢查輸入訊息", "red")
            # continue
            msg = rospy.wait_for_message("/detect_grasps/clustered_grasps", GraspConfigList)  # 等待接收抓取姿態消息

            position_in_base_list,sample_in_base_list,class_in_base_list,have_grasps=grasp_callback(msg,voxelize_pcd)  # 處理接收到的消息 
            if input("是否重來？(y/n): ").strip().lower() == 'y':
                cloud_publisher.publish(o3d_to_ros_pointcloud(combined_pcd))
                color_print("發送點雲", "blue")
            else:
                break
            # ros_cloud=o3d_to_ros_pointcloud(pcd = o3d.io.read_point_cloud("/home/chen/catkin_ws/src/pcl_with_gpd/scripts/test.ply") , frame_id="base")
            
        
        
        for position ,sample,classes in zip(position_in_base_list,sample_in_base_list,class_in_base_list):
            if classes is not None:
                
                best_position= position
                best_sample= sample
                best_class=classes 
                break
            else: color_print("無法分類，尋找下一個姿態！", "yellow") 

        # exit()

        # move_script_with_monitor(PREPARE_POSE,motion_type='PTP')
        input(f'NEXT.................')
        move_script_with_monitor(best_position,motion_type='PTP')
        
        move_script_with_monitor(best_sample)
        gripper_pick(pick_distance=255,inital_bool=True)
        rospy.sleep(2)  # 等待夾爪閉合
        move_script_with_monitor(best_position,motion_type='PTP')
        # up_pose=best_position[:2]+[200]+best_position[3:]
        # move_script_with_monitor(up_pose)
        move_script_with_monitor(position[:2]+[385]+best_position[3:])


        obj_detect=get_gripper_position()
        if obj_detect !=2:
            
            color_print(f"obj_detect={obj_detect},🛠 檢測到夾爪未完全閉合，正在閉合夾爪...", "red")
            gripper_pick(pick_distance=0,inital_bool=True)
            continue
        else:
            color_print("✅ 夾爪已經閉合", "green")
            

        if best_class == "aluextru":
            color_print(best_class, "red")
            move_script_with_monitor(PUT_POSE_0)
            move_script_with_monitor(PUT_DOWN_POSE_0)  
            gripper_pick(pick_distance=0,inital_bool=True)
            move_script_with_monitor(PUT_POSE_0)
    
        elif best_class == "bin":
            color_print(best_class, "green")
            move_script_with_monitor(PUT_POSE_1)
            # move_script_with_monitor(PUT_POSE_1[:2]+[161.9]+PUT_POSE_1[3:])
            move_script_with_monitor(PUT_DOWN_POSE_1)
            gripper_pick(pick_distance=0,inital_bool=True)
            move_script_with_monitor(PUT_POSE_1)
            
        elif best_class == "twpipe":
            color_print(best_class, "blue")
            move_script_with_monitor(PUT_POSE_2)
            # move_script_with_monitor(PUT_POSE_2[:2]+[161.9]+PUT_POSE_2[3:])
            move_script_with_monitor(PUT_DOWN_POSE_2)
            gripper_pick(pick_distance=0,inital_bool=True)
            move_script_with_monitor(PUT_POSE_2)
        else:
            color_print(best_class, "yellow")

        print("下一輪夾取...")
        input('next round .................')
        
        # cv2.destroyAllWindows()
    pipeline.stop()

