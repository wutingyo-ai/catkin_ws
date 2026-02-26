from realsense import *
import pyrealsense2 as rs
import cv2
import open3d as o3d
import numpy as np

""" while True:
    frame = Get_RGB_Frame()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("s", frame)    
    cv2.waitKey(1) """



pc = o3d.geometry.PointCloud()
pointcloud = Get_PointCloud(is_temporal_filter = True, sample_length=20)
pc.points = o3d.utility.Vector3dVector(pointcloud)
print(np.asarray(pc.points).shape[0])
# np.savetxt('data.csv', pointcloud, delimiter=',', fmt='%.4f', header='X,Y,Z', comments='')

# print(pointcloud[2000:2100])
o3d.visualization.draw_geometries([pc])

# # Create a context object. This object owns the handles to all connected realsense devices
# pipeline = rs.pipeline()

# # Configure streams
# config = rs.config()


# # Start streaming
# pipeline.start(config)


# def Get_Depth_Frame():
#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     depth_image = np.asanyarray(depth_frame.get_data())
#     colorizer = rs.colorizer()
#     colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data()) #BGR
#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#     cv2.imshow('depth',colorized_depth) 
#     cv2.waitKey(1000)
#     cv2.destroyAllWindows() 
#     return depth_image

# # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


# try:
    
#     """ frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()
#     # Convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())

#     colorizer = rs.colorizer()
#     colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data()) #BGR

#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#     print(colorized_depth[100,100,:])
#     print('\n')
#     print(depth_image[100,100])
    
    
#     cv2.imshow('depth',colorized_depth) 
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()  """

# finally:
#     pipeline.stop()
    







