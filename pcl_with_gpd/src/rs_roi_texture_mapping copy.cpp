#include <librealsense2/rs.hpp>
#include "example.hpp"
#include <opencv2/opencv.hpp> // OpenCV for image processing
#include <iostream>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

// void register_glfw_callbacks(window &app, glfw_state &app_state);

int main(int argc, char *argv[])
try
{
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
    viewer->setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
    // viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, -1, 0);
    // 定義對齊物件

    int frame_count = 0;

    while (!viewer->wasStopped()) // while (app)
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

        pc.map_to(color_frame);
        auto points = pc.calculate(depth_frame);

        // 將 RealSense 影像轉換為 OpenCV Mat 格式
        int width = color_frame.as<rs2::video_frame>().get_width();
        int height = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat color_mat(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR); // 轉換為 OpenCV BGR 格式

        // 定義選取區域 (ROI)
        cv::Rect roi(300, 200, 400, 300);                        // x, y, width, height
        cv::rectangle(color_mat, roi, cv::Scalar(0, 255, 0), 2); // 繪製矩形框

        //  // 顯示 ROI 的顏色影像
        cv::Mat roi_image = color_mat(roi); // 切出 ROI
        // cv::imshow("ROI Image", roi_image); // 顯示 ROI 圖像

        // 顯示影像
        cv::imshow("Color Image with ROI", color_mat);
        if (cv::waitKey(1) == 27)
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

        roi_cloud->clear(); // 清除舊的點雲資料
        // 3. 遍歷 ROI 區域，轉換深度資訊為 3D 座標並添加 RGB 顏色
        for (int y = roi.y; y < roi.y + roi.height; ++y)
        {
            for (int x = roi.x; x < roi.x + roi.width; ++x)
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
                    pcl_point.r = 0;
                    pcl_point.g = 0;
                    pcl_point.b = 0;
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
        /* roi_cloud->clear(); // 清除舊的點雲資料

        // 3. 遍歷整張影像區域，轉換深度資訊為 3D 座標並添加 RGB 顏色
        for (int y = 0; y < color_mat.rows; ++y)
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
                    pcl_point.r = 255;
                    pcl_point.g = 0;
                    pcl_point.b = 0;
                    // printf("r=%d\n",pcl_point.r );
                    // uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    //                 static_cast<uint32_t>(g) << 8 |
                    //                 static_cast<uint32_t>(b));
                    // pcl_point.rgb = *reinterpret_cast<float *>(&rgb); // 轉換格式

                    roi_cloud->points.emplace_back(pcl_point);
                }
            }
        } */

        /* // 過濾 ROI 內的點雲
        for (int y = roi.y; y < roi.y + roi.height; ++y)
        {
            for (int x = roi.x; x < roi.x + roi.width; ++x)
            {
                int index = y * width + x;
                float depth_value = depth_frame.get_distance(x, y);

                if (depth_value > 0)
                { // 確保深度有效
                    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    float point[3];
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_value);
                    roi_cloud->points.emplace_back(point[0], point[1], point[2]);
                }
            }
        } */

        /* // **更新 PCL 視覺化**
        if (!viewer->updatePointCloud(roi_cloud, "roi_cloud"))
            viewer->addPointCloud(roi_cloud, "roi_cloud"); */
        frame_count++;
        // printf("frame_count=%d\n", frame_count);
        // 8. PCL 視覺化 (更新顏色點雲)
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(roi_cloud);
        if (!viewer->updatePointCloud(roi_cloud, rgb, "roi_cloud"))
        {
            viewer->addPointCloud(roi_cloud, rgb, "roi_cloud");
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "roi_cloud");

        /* if (!viewer->updatePointCloud<pcl::PointXYZRGB>(roi_cloud, "roi_cloud"))
        {
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(roi_cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(roi_cloud, "roi_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "roi_cloud"); // 設定點大小
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1.0, 1.0, "roi_cloud"); // 啟用 RGB
        } */
        // 添加坐標軸
        viewer->addCoordinateSystem(0.1);

        roi_cloud->width = roi_cloud->points.size();
        roi_cloud->height = 1;

        viewer->spinOnce(10); // **讓視窗刷新，而不會卡住程式**

        // // 切出 ROI 中的點雲
        // for (int y = roi.y; y < roi.y + roi.height; ++y)
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
