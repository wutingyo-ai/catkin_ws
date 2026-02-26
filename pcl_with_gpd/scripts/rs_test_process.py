# from RealSense.realsense import *
import pyrealsense2 as rs
import cv2
import open3d as o3d
import numpy as np
import rospy
# import pcl_msgs
import matplotlib.pyplot as plt


pipeline = rs.pipeline()
config = rs.config()
pipeline.start(config)
pc = rs.pointcloud()


def Get_Depth_Frame():
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())  # BGR
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imshow('depth', colorized_depth)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()
    return depth_image

def Get_PointCloud(
                   depth=np.inf,
                   sample_length=10, 
                   is_decimation_filter=False, 
                   is_spatial_filter=False, 
                   is_temporal_filter=False, 
                   is_hole_filling_filter=False,
                   is_depth_to_disparity=False,
                   is_disparity_to_depth=False):
    # output realsense point cloud as a np array(nx3)
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
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
    filtered_verts=verts[verts[:,2]<depth]

    return filtered_verts

def process_pointcloud(pointcloud):
    # 將 NumPy 陣列轉換為 Open3D 點雲
    pcd = o3d.geometry.PointCloud()
    # pointcloud = Get_PointCloud(is_temporal_filter = True, sample_length=20)
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    # 體素化濾波器
    voxel_size = 0.01  # 設定體素大小
    pcd_voxel = pcd.voxel_down_sample(voxel_size)
    
    # 點雲分群
    labels = np.array(pcd_voxel.cluster_dbscan(eps=0.5, min_points=10, print_progress=True))

    np_pcd_voxel=np.asarray(pcd_voxel.points)

    

    # 根據標籤對點雲排序
    sorted_indices = np.argsort(labels)
    sorted_points = np_pcd_voxel[sorted_indices]
    sorted_labels = labels[sorted_indices]

    # print("Sorted Points:\n", sorted_points)
    # print("Sorted Labels:\n", sorted_labels)

    

    return sorted_points, sorted_labels  # 返回處理後的點雲和分群標籤



def visualization(point_cloud):  #numpy point cloud 
    pcd = o3d.geometry.PointCloud()
    # pointcloud = Get_PointCloud(is_temporal_filter = True, sample_length=30)
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    
    # o3d.visualization.draw_geometries([pcd],zoom=0.2,
    #                               front=[0.4257, -0.2125, -0.8795],
    #                               lookat=[2.6172, 2.0475, 1.532],
    #                               up=[-0.0694, -0.9768, 0.2024])
    o3d.visualization.draw_geometries([pcd],
                                    zoom=5,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])    

def cluster_to_visual(point_cloud,labels):
    pcd = o3d.geometry.PointCloud()
    # pointcloud = Get_PointCloud(depth=2,is_temporal_filter = True, sample_length=30)
    pcd.points = o3d.utility.Vector3dVector(point_cloud)


    # voxel_pcd = pcd.voxel_down_sample(voxel_size=0.06)
    
    
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("rainbow")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd],
                                    zoom=5,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])    


    
if __name__ == '__main__':
    try:
        np_pointcloud = Get_PointCloud(depth=2,is_temporal_filter = True, sample_length=30)
        pcd,label=process_pointcloud(np_pointcloud)

        # pcd=pcd[label==0]
        # cluster_to_visual(pcd,label)
        cluster_to_visual(pcd,label)
        
        # print(f"\n labels={label}")
        # pcd.sort(label)
        

    except :
        pass







