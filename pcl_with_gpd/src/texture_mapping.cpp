#include <librealsense2/rs.hpp>
#include "example.hpp"
#include <opencv2/opencv.hpp> // OpenCV for image processing
#include <iostream>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h> // 包含 StatisticalOutlierRemoval 的標頭
#include <pcl/filters/voxel_grid.h>                  // 包含 VoxelGrid 的標頭
#include <pcl/features/normal_3d.h>                  // 包含 NormalEstimation 的標頭
#include <ros/ros.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudSources.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // 用於轉換 PCL 和 ROS 點雲格式
#include <std_msgs/Int64.h>
#include <geometry_msgs/Point.h>
#include <tuple>

// 滑鼠回調函式
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE) // 只在滑鼠移動時觸發
    {
        cv::Mat *img = reinterpret_cast<cv::Mat *>(userdata);   // 取得傳入的影像指標
        if (x >= 0 && y >= 0 && x < img->cols && y < img->rows) // 確保座標在範圍內
        {
            cv::Vec3b pixel = img->at<cv::Vec3b>(y, x); // OpenCV 影像存取方式 (注意 y 是 row, x 是 col)
            cout << "Position: (" << x << ", " << y << ") "
                 << "RGB: (" << (int)pixel[2] << ", " << (int)pixel[1] << ", " << (int)pixel[0] << ")" << endl;
        }
    }
}

std::tuple<int, int, int, int> boundingBoxToXYWH(const std::pair<int, int> &topLeft, const std::pair<int, int> &bottomRight)
{
    int x1 = topLeft.first;      // 左上點的 x 坐標
    int y1 = topLeft.second;     // 左上點的 y 坐標
    int x2 = bottomRight.first;  // 右下點的 x 坐標
    int y2 = bottomRight.second; // 右下點的 y 坐標

    // 計算寬度和高度
    int width = x2 - x1;
    int height = y2 - y1;

    // 返回 x, y, width, height
    return std::make_tuple(x1, y1, width, height);
}
// void register_glfw_callbacks(window &app, glfw_state &app_state);

int main(int argc, char *argv[])
try
{
    /* // 初始化 ROS 節點
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    // 創建 Publisher
    ros::Publisher cloud_samples_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_stitched", 1); */

    // ros::Publisher cloud_samples_pub_2 = nh.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 1);

    // // 建立 OpenGL 窗口
    // window app(1280, 720, "RealSense Pointcloud ROI Example");
    // glfw_state app_state;
    // register_glfw_callbacks(app, app_state);

    // RealSense pipeline 設定
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    pipe.start();
    // PCL 視覺化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ROI Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    // 手動畫出 XYZ 軸
    pcl::PointXYZ origin(0, 0, 0);
    pcl::PointXYZ x_axis(0.1, 0, 0);
    pcl::PointXYZ y_axis(0, 0.1, 0);
    pcl::PointXYZ z_axis(0, 0, 0.1);

    // 紅色 X 軸
    viewer->addLine(origin, x_axis, 1.0, 0.0, 0.0, "x_axis");
    // 綠色 Y 軸
    viewer->addLine(origin, y_axis, 0.0, 1.0, 0.0, "y_axis");
    // 藍色 Z 軸
    viewer->addLine(origin, z_axis, 0.0, 0.0, 1.0, "z_axis");

    // 調整視角，修正顛倒問題
    viewer->setCameraPosition(0, 0, 0.2, 0, 0, 1, 0, -1, 0);
    // viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, -1, 0);
    // 定義對齊物件

    int frame_count = 0;
    while (!viewer->wasStopped()) // while (app)
    // while(0<=frame_count && frame_count<10)
    {
        rs2::align align(RS2_STREAM_COLOR); // 使用正確的流類型
        auto frames = pipe.wait_for_frames();
        auto aligned_frames = align.process(frames); // 對齊幀
        auto color_frame = aligned_frames.get_color_frame();
        auto depth_frame = aligned_frames.get_depth_frame();

        // // 檢查相機是否有 RGB 影像
        // if (!color_frame)
        // {
        //     color_frame = frames.get_infrared_frame();
        //     cout << "no frame";
        //     printf("no frame");
        // }
        // 設定滑鼠回調函式

        pc.map_to(color_frame);
        auto points = pc.calculate(depth_frame);

        // 將 RealSense 影像轉換為 OpenCV Mat 格式
        int width = color_frame.as<rs2::video_frame>().get_width();
        int height = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat color_mat(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR); // 轉換為 OpenCV BGR 格式

        // 查看像素與位置
        cv::setMouseCallback("Color Image with ROI", onMouse, &color_mat);

        std::pair<int, int> topLeftPoint = {430, 142};     // 左上點
        std::pair<int, int> bottomRightPoint = {577, 305}; // 右下點
        auto [x_roi, y_roi, width_roi, height_roi] = boundingBoxToXYWH(topLeftPoint, bottomRightPoint);
        // std::cout << "x_roi: " << x_roi << ", y_roi: " << y_roi << ", width_roi: " << width_roi << ", height_roi: " << height_roi << std::endl;
        // break;

        // 定義選取區域 (ROI)                     // x, y, width, height
        cv::Rect roi(x_roi, y_roi, width_roi, height_roi);
        cv::rectangle(color_mat, roi, cv::Scalar(0, 255, 0), 2); // 繪製矩形框

        //  // 顯示 ROI 的顏色影像
        cv::Mat roi_image = color_mat(roi); // 切出 ROI
        // cv::imshow("ROI Image", roi_image); // 顯示 ROI 圖像

        // // 顯示影像
        cv::imshow("Color Image with ROI", color_mat);
        if (cv::waitKey(50) == 27)
            break; // 按 ESC 退出

        // 確保幀有效
        if (!depth_frame || !color_frame)
        {
            continue; // 跳過無效幀
        }

        // 生成點雲
        const auto points_vertices = points.get_vertices(); // 獲取所有點雲座標
        // std::vector<rs2::vertex> roi_points;
        // 在切出 ROI 中的點雲後
        // pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 1. 建立帶 RGB 顏色的點雲
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // roi_cloud->points.clear();
        auto intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        


        // for (int i = 0; i < RS2_DISTORTION_COUNT; i++)
        // {
        //     printf("%f ", intrinsics.coeffs[i]); // 畸變係數
        // }
        roi_cloud->clear(); // 清除舊的點雲資料
        // 3. 遍歷 ROI 區域，轉換深度資訊為 3D 座標並添加 RGB 顏色
        for (int y = roi.y; y < roi.y + roi.height; ++y)
        {
            for (int x = roi.x; x < roi.x + roi.width; ++x)
            {
                int index = y * width + x;
                float depth_value = depth_frame.get_distance(x, y); // 取得深度值

                if (depth_value > 0 && depth_value<3) // 確保深度有效 0.69 m 深度
                {
                    // 4. 轉換 2D 像素座標為 3D 座標
                    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    float point[3];
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_value);

                    // 5. 取得對應的 RGB 值
                    cv::Vec3b color = color_mat.at<cv::Vec3b>(y, x);
                    uint8_t r = color[2]; // OpenCV 預設 BGR，因此需要調換順序
                    uint8_t g = color[1];
                    uint8_t b = color[0];

                    // 6. 建立 `PointXYZRGB` 並存入點雲
                    pcl::PointXYZRGB pcl_point;
                    pcl_point.x = point[0];
                    pcl_point.y = point[1];
                    pcl_point.z = point[2];

                    // 設置顏色（PCL 的 RGB 需轉為 float 類型）
                    // 6. 設定顏色（PCL RGB 需轉換成 float 格式）
                    pcl_point.r = r;
                    pcl_point.g = g;
                    pcl_point.b = b;
                    // printf("r=%d\n",pcl_point.r );
                    // uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    //                 static_cast<uint32_t>(g) << 8 |
                    //                 static_cast<uint32_t>(b));
                    // pcl_point.rgb = *reinterpret_cast<float *>(&rgb); // 轉換格式

                    roi_cloud->points.emplace_back(pcl_point);
                    // printf("pcl_point_r=%d\n", pcl_point.r);
                }
            }
        }

        /* // 使用 Statistical Outlier Removal 濾波器處理雜訊
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(roi_cloud);
        sor.setMeanK(50);               // 設置鄰域大小 (可調整)
        sor.setStddevMulThresh(1.0);    // 設置雜訊閾值 (可調整)
        sor.filter(*filtered_roi_cloud); // 濾波後的點雲 */

        /* // 添加 Voxel Downsampling
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        float voxel_size = 0.01f; // 設定 voxel 大小 (可根據需要調整)
        voxel_filter.setInputCloud(filtered_roi_cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size); // 設定每個 voxel 的大小
        voxel_filter.filter(*voxel_filtered_cloud); // 濾波後的點雲

        std::cout << "原始點雲點數: " << roi_cloud->points.size()
                  << ", 濾波後點數: " << filtered_roi_cloud->points.size()
                  << ", Voxel Downsampled 點數: " << voxel_filtered_cloud->points.size() << std::endl; */

        /*  // 添加法向量估計
         pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
         pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
         ne.setInputCloud(voxel_filtered_cloud);

         // 使用 KNN 搜尋法來計算法向量
         pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
         ne.setSearchMethod(tree);
         ne.setRadiusSearch(0.03); // 設定半徑搜尋範圍 (可根據需要調整)
         ne.compute(*normals); // 計算法向量

         std::cout << "法向量數量: " << normals->size() << std::endl; */

        // roi_cloud->clear(); // 清除舊的點雲資料

        // 3. 遍歷整張影像區域，轉換深度資訊為 3D 座標並添加 RGB 顏色
        /* for (int y = 0; y < color_mat.rows; ++y)
        {
            for (int x = 0; x < color_mat.cols; ++x)
            {
                int index = y * width + x;
                float depth_value = depth_frame.get_distance(x, y); // 取得深度值

                if (depth_value > 0) // 確保深度有效
                {
                    // 4. 轉換 2D 像素座標為 3D 座標
                    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    float point[3];
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_value);

                    // 5. 取得對應的 RGB 值
                    cv::Vec3b color = color_mat.at<cv::Vec3b>(y, x);
                    uint8_t r = color[2]; // OpenCV 預設 BGR，因此需要調換順序
                    uint8_t g = color[1];
                    uint8_t b = color[0];

                    // 6. 建立 `PointXYZRGB` 並存入點雲
                    pcl::PointXYZRGB pcl_point;
                    pcl_point.x = point[0];
                    pcl_point.y = point[1];
                    pcl_point.z = point[2];

                    // 設置顏色（PCL 的 RGB 需轉為 float 類型）
                    // 6. 設定顏色（PCL RGB 需轉換成 float 格式）
                    pcl_point.r = r;
                    pcl_point.g = g;
                    pcl_point.b = b;
                    // printf("r=%d\n",pcl_point.r );
                    // uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    //                 static_cast<uint32_t>(g) << 8 |
                    //                 static_cast<uint32_t>(b));
                    // pcl_point.rgb = *reinterpret_cast<float *>(&rgb); // 轉換格式

                    roi_cloud->points.emplace_back(pcl_point);
                }
            }
        } */

        frame_count++;
        auto visulalize_point_cloud = roi_cloud;
        // // 打印內參矩陣
        // printf("Camera Intrinsics:\n");
        // printf("Width: %d\n", intrinsics.width);
        // printf("Height: %d\n", intrinsics.height);
        // printf("PPX: %f\n", intrinsics.ppx);     // 主點 x 坐標
        // printf("PPY: %f\n", intrinsics.ppy);     // 主點 y 坐標
        // printf("FX: %f\n", intrinsics.fx);       // 焦距 x
        // printf("FY: %f\n", intrinsics.fy);       // 焦距 y
        // printf("Model: %d\n", intrinsics.model); // 相機模型
        // printf("Distortion: ");

        // printf("frame_count=%d\n", frame_count);
        // 8. PCL 視覺化 (更新顏色點雲) - 使用經過濾波的點雲
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(visulalize_point_cloud);
        if (!viewer->updatePointCloud(visulalize_point_cloud, rgb, "visulalize_point_cloud"))
        {
            viewer->addPointCloud(visulalize_point_cloud, rgb, "visulalize_point_cloud");
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "visulalize_point_cloud");

        // 使用 addPointCloudNormals 方法將法向量添加到視覺化器
        std::string normals_id = "normals_" + std::to_string(frame_count); // 使用唯一的 ID
        // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(visulalize_point_cloud, normals, 1, 0.1, normals_id);
        // 添加坐標軸
        viewer->addCoordinateSystem(0.1);

        visulalize_point_cloud->width = visulalize_point_cloud->points.size();
        visulalize_point_cloud->height = 1;

        viewer->spinOnce(10); // **讓視窗刷新，而不會卡住程式**

        /* // 將 filtered_roi_cloud 轉換為 ROS 的 PointCloud2 格式
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_roi_cloud, output);
        output.header.frame_id = "base_link"; // 設定座標框架 (根據您的需求調整)
        output.header.stamp = ros::Time::now(); // 設定時間戳 */

        /*  // 創建 CloudSources 消息
         gpd_ros::CloudSources cloud_sources;
         cloud_sources.cloud = output; // 將點雲賦值給 cloud_sources

         // Create an Int64 message for the camera source
         std_msgs::Int64 camera_index;
         camera_index.data = 0; // Set the camera index to 0
         cloud_sources.camera_source.push_back(camera_index); // Push the Int64 message

         // Add the camera position
         geometry_msgs::Point view_point;
         view_point.x = 0.0;
         view_point.y = 0.0;
         view_point.z = 0.0;
         cloud_sources.view_points.push_back(view_point); // Push the camera position

         // 創建 CloudSamples 消息
         gpd_ros::CloudSamples cloud_samples;
         cloud_samples.cloud_sources = cloud_sources;

         // 添加樣本點 (這裡假設您有一些樣本點)
         geometry_msgs::Point sample_point;
         sample_point.x = 0.5; // 示例樣本點
         sample_point.y = 0.5;
         sample_point.z = 0.5;
         cloud_samples.samples.push_back(sample_point); */

        // 發送 CloudSamples 消息到指定的 topic
        // cloud_samples_pub.publish(output);
        // std::cin.get();
        // break;

        // // 切出 ROI 中的點雲
        // for (int y = roi.y; y < roi.y + roi.height; ++y)/
        // {
        //     for (int x = roi.x; x < roi.x + roi.width; ++x)
        //     {
        //         int index = y * width + x;
        //         float depth_value = depth_frame.get_distance(x, y); // 獲取該像素的深度值

        //         if (depth_value > 0) { // 確保深度值有效
        //             // 獲取相機內部參數
        //             auto intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        //             // 將 2D 像素座標轉換為 3D 空間座標
        //              // 將 2D 像素座標轉換為 3D 空間座標
        //             float pixel[2] = { static_cast<float>(x), static_cast<float>(y) }; // 將像素座標存儲在 float 陣列中
        //             float point[3]; // 用於存儲 3D 坐標
        //             rs2_deproject_pixel_to_point(point, & intrinsics, pixel, depth_value);
        //             roi_points.push_back(rs2::vertex{point[0], point[1], point[2]}); // 將 3D 坐標存入 roi_points

        //             // rs2::vertex v = rs2_deproject_pixel_to_point(intrinsics, {x, y}, depth_value);
        //             // roi_points.push_back(v);
        //         }
        //     }
        // }

        // 現在 roi_points 中包含了 ROI 區域的點雲

        // 顯示點雲
        // app_state.tex.upload(color_frame);
        // draw_pointcloud(app.width(), app.height(), app_state, points);

        // // 將 roi_points 轉換為 PCL 點雲格式
        // for (const auto& vertex : roi_points) {
        //     roi_cloud->points.emplace_back(vertex.x, vertex.y, vertex.z);
        // }
        // roi_cloud->width = roi_cloud->points.size();
        // roi_cloud->height = 1; // 設置為 1 以表示非結構化點雲

        // // 使用 PCL 的可視化工具顯示點雲
        // pcl::visualization::CloudViewer viewer("ROI Point Cloud Viewer");
        // viewer.showCloud(roi_cloud);

        // // 等待直到視窗關閉
        // while (!viewer.wasStopped()) {
        //     // 可以在這裡添加其他代碼
        // }
        // if (frame_count==100){viewer->close();}
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
