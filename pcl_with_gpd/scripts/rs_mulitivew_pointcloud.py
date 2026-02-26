#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import math
import rospy
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import AskItem, AskItemRequest
from scipy.spatial.transform import Rotation as R_scipy
import re
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2


# T_cam2gripper=np.array(
# [[-0.907,-0.416,-0.061,0.078],
# [0.421,-0.900,-0.112,-0.071],
# [-0.008,-0.127,0.992,-0.160],
# [0.000,0.000,0.000,1.000]])



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

def new_monitor(monitor_target_point:list)->list:
    global robot_pose
    rate = rospy.Rate(100)
    arrive = 0
    # monitor_target_point=construct_point(monitor_target_point)
    point_offset=0.02
    # while rospy.is_shutdown()==False:
    # 使用 wait_for_message 直接獲取最新的 FeedbackState 訊息
    msg = rospy.wait_for_message('feedback_states', FeedbackState)
    
    # 提取 tool_pose 資訊並轉換為數值列表
    if len(msg.tool0_pose) == 6:
        robot_pose = [
            msg.tool0_pose[0]*1000,  # x msg unit:m
            msg.tool0_pose[1]*1000,  # y msg unit:m
            msg.tool0_pose[2]*1000,  # z msg unit:m
            math.degrees(msg.tool0_pose[3]),  # rx msg unit:rad
            math.degrees(msg.tool0_pose[4]),  # ry msg unit:rad
            math.degrees(msg.tool0_pose[5])   # rz msg unit:rad
        ]
        # print(robot_pose)
    else:
        rospy.logerr("Invalid tool0_pose length")
            # continue

        # # 計算 robot_pose 與目標點位的變化程度
        # change_of_point = abs((np.array(monitor_target_point[:3]) - np.array(robot_pose[:3])))
        # # print(robot_pose[:3])
        
        # if abs(monitor_target_point[0]-robot_pose[0])<point_offset and abs(monitor_target_point[1]-robot_pose[1])<point_offset and abs(monitor_target_point[2]-robot_pose[2])<point_offset:
        #     arrive+=1
        #     rospy.loginfo(str(change_of_point[:3]))
            
        # elif abs(monitor_target_point[0]-robot_pose[0])<point_offset and abs(monitor_target_point[1]-robot_pose[1])<point_offset and abs(monitor_target_point[2]-robot_pose[2])<point_offset:
        #     arrive=0
        
        # if arrive>=3:
        #     # rospy.loginfo(str(change_of_point[:3]))
        #     break

        # rate.sleep()
        # # print('change of point =', change_of_point)
    
    print('Monitor Finish!')


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

def capture_pointcloud(frames, T_flange2base, T_cam2gripper, align, pc)-> o3d.geometry.PointCloud:
    aligned = align.process(frames)
    depth = aligned.get_depth_frame()
    color = aligned.get_color_frame()
    if not depth or not color: return None

    pc.map_to(color)
    points = pc.calculate(depth)
    vtx = np.asanyarray(points.get_vertices())
    tex = np.asanyarray(points.get_texture_coordinates())
    xyz = np.array([[v[0], v[1], v[2]] for v in vtx])
    xyz_homo = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
    xyz_base = (T_flange2base @ T_cam2gripper @ xyz_homo.T).T[:, :3]

    img = np.asanyarray(color.get_data())
    h, w = img.shape[:2]
    img = img[:, :, :]  # BGR → RGB

    rgb = np.array([
        img[min(max(int(t[1]*h), 0), h-1), min(max(int(t[0]*w), 0), w-1)] / 255.0
        for t in tex
    ])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_base)
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    return pcd

def preprocess_point_cloud(pcd, voxel_size=0.005)->o3d.geometry.PointCloud:
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


                
if __name__ =='__main__':

    rospy.init_node('node')
    
    # robot_pose
    # 初始化 RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    pipeline.start(config)

    # 對齊 depth 到 color
    align = rs.align(rs.stream.color)

    # 建立點雲物件
    pc = rs.pointcloud()
    robot_pose=[]
    np.set_printoptions(suppress=True, precision=6)
    T_cam2gripper=np.array(
    [[-0.995,0.099,-0.017,0.038],
    [-0.098,-0.992,-0.079,-0.075],
    [-0.025,-0.076,0.997,0.038],
    [0.000,0.000,0.000,1.000]])

    pcd_combined = None
    try:
        for i in range(2):
            input(f"\n📸 移動到第 {i+1} 視角後按 Enter 擷取...")
            
            for _ in range(10): 
                frames = pipeline.wait_for_frames()
            
            pose = ask_flange_pose()
            T_flange2base = pose_to_matrix(pose)

            pcd = capture_pointcloud(frames, T_flange2base, T_cam2gripper, align, pc)
            if pcd is None: continue
            pcd = preprocess_point_cloud(pcd)
            if pcd_combined is None:
                pcd_combined = pcd
            else:
                T_icp, _ = icp_align_robust_plane(pcd, pcd_combined)
                pcd.transform(T_icp)
                pcd_combined += pcd

    finally:
        pipeline.stop()

    print("✅ 顯示配準後結果")
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    
    # 設定視角參數
    zoom = 0.01
    
    front = [0.0, -0.1, 0.0]
    lookat = [0.0, 0.0, 0.0]
    up = [0.0, 0.0, 1.0]

    msg=o3d_to_ros_pointcloud(pcd_combined)
    
    # 顯示點雲和座標系
    o3d.visualization.draw_geometries([pcd_combined, axis],
                                    zoom=zoom,
                                    front=front,
                                    lookat=lookat,
                                    up=up,
                                    window_name="RGB PointCloud from RealSense")
    
    
    
    # exit()
    

'''
    Point2base_stitch = np.empty(shape=(0,3))
    rgb_list_stitch = np.empty(shape=(0,3))
    try:
        for frames_count in range(5):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
                
            # 建立點雲貼圖對應
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            if frames_count>1:
                # 轉成 numpy 陣列
                vtx = np.asanyarray(points.get_vertices())
                tex = np.asanyarray(points.get_texture_coordinates())
                # print(vtx.shape,tex.shape)
                xyz = np.array([[v[0], v[1], v[2]] for v in vtx])  # N x 3
                # print('xyz',xyz)
                # xyz: shape (N, 3)
                ones = np.ones((xyz.shape[0], 1))         # shape (N, 1)
                xyz_homo = np.hstack((xyz, ones))
                # print('xyz_homo',xyz_homo)  
                # shape (N, 4)

                ask_flange_pose()
                print('robot_pose',robot_pose)
                print(pose_to_matrix(robot_pose))
                
                T_flange2base = pose_to_matrix(robot_pose)
                Point2base = (T_flange2base @ T_cam2gripper @ xyz_homo.T).T[:,:3]
                Point2base_stitch = np.vstack((Point2base_stitch,Point2base))
                
                print('p2b',Point2base_stitch.shape)
                
                
                color_image = np.asanyarray(color_frame.get_data())
                h, w, _ = color_image.shape

                rgb = []
                for t in tex:
                    u = min(max(int(t[0] * w), 0), w - 1)
                    v = min(max(int(t[1] * h), 0), h - 1)
                    rgb.append(color_image[v, u] / 255.0)  # Open3D expects float RGB (0~1)

                rgb = np.array(rgb)  # N x 3

                rgb_list_stitch = np.vstack((rgb_list_stitch,rgb))
                
                
            input('stop')
            # break  # 顯示一次就退出（你可改成 while 顯示更新）

    finally:
        
        pipeline.stop()
    # 建立 Open3D 點雲物件
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(Point2base_stitch)
        pcd.colors = o3d.utility.Vector3dVector(rgb_list_stitch)

        
        
    # 建立座標系
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

        # 設定視角參數
        zoom = 0.5
        front = [0.0, -1.0, 0.0]
        lookat = [0.0, 0.0, 0.0]
        up = [0.0, 0.0, 1.0]

        # 顯示點雲和座標系
        o3d.visualization.draw_geometries([pcd, axis],
                                        zoom=zoom,
                                        front=front,
                                        lookat=lookat,
                                        up=up,
                                        window_name="RGB PointCloud from RealSense")
        # break  # 顯示一次就退出（你可改成 while 顯示更新）'''