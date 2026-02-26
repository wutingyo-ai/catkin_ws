import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================
# 1️⃣ 初始化 RealSense
# ============================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# 創建點雲物件
pc = rs.pointcloud()
points = rs.points()

# ============================
# 2️⃣ 設定 Matplotlib 即時顯示
# ============================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 設定顯示範圍
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 1)

sc = ax.scatter([], [], [], s=1, c=[], cmap='jet', marker='o')

plt.ion()  # 啟用互動模式
plt.show()

# ============================
# 3️⃣ 即時獲取點雲並更新畫面
# ============================
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue
        
        # 產生點雲
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())  # 取得 3D 座標 (XYZ)

        # 過濾無效點
        vtx = np.array([[p[0], p[1], p[2]] for p in vtx if p[2] > 0])
        if len(vtx) == 0:
            continue

        # 分離 X, Y, Z 軸資料
        X, Y, Z = vtx[:, 0], vtx[:, 1], vtx[:, 2]

        # 更新 Matplotlib 圖表
        sc._offsets3d = (X, Y, Z)
        sc.set_array(Z)  # 以 Z 軸作為顏色
        plt.draw()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("關閉 RealSense...")
    pipeline.stop()
    plt.ioff()
    plt.show()
