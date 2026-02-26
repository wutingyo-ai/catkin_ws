// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/point_types.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderer.h>
// #include <vtkRenderWindowInteractor.h>
// #include <QVTKWidget.h>  // QVTKWidget 標頭檔
// #include <QApplication>
// #include <QMainWindow>
// #include <QVBoxLayout>

// void closeViewer(pcl::visualization::PCLVisualizer::Ptr viewer, vtkSmartPointer<vtkRenderWindow> renderWindow) {
//     viewer->close();  // 讓 PCL Viewer 關閉
//     renderWindow->Finalize();  // 釋放 VTK 視窗資源
//     viewer.reset();  // 釋放記憶體
// }

// int main(int argc, char** argv) {
//     // 初始化 Qt 應用程式
//     QApplication app(argc, argv);

//     // === 創建主窗口 ===
//     QMainWindow mainWindow;
//     QWidget *centralWidget = new QWidget();
//     QVBoxLayout *layout = new QVBoxLayout();
//     centralWidget->setLayout(layout);
//     mainWindow.setCentralWidget(centralWidget);

//     // === 創建 QVTKWidget 與 VTK 視窗 ===
//     QVTKWidget *vtkWidget = new QVTKWidget();
//     layout->addWidget(vtkWidget);

//     vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//     vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
//     renderWindow->AddRenderer(renderer);
    
//     // === 使用 QVTKInteractor 替代 vtkRenderWindowInteractor ===
//     vtkSmartPointer<QVTKInteractor> interactor = vtkSmartPointer<QVTKInteractor>::New();
//     interactor->SetRenderWindow(renderWindow);  // 將交互器綁定到 VTK 視窗

//     pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "Viewer"));
//     viewer->setBackgroundColor(0.1, 0.1, 0.1);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud->width = 100;
//     cloud->height = 1;
//     cloud->points.resize(cloud->width * cloud->height);
//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//     }
//     viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

//     // === 綁定 VTK 視窗和 QVTKWidget ===
//     vtkWidget->SetRenderWindow(renderWindow);
//     renderWindow->SetInteractor(interactor);

//     // 啟動事件循環
//     viewer->spinOnce(100);
    
//     // 顯示窗口
//     mainWindow.show();
    
//     // 等待 Qt 應用程式結束
//     return app.exec();
// }

















#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <chrono>
#include <thread>

// 自訂關閉函數，確保 VTK 視窗完全關閉
void closeViewer(pcl::visualization::PCLVisualizer::Ptr viewer, vtkSmartPointer<vtkRenderWindow> renderWindow) {
    // viewer->close();  // 讓 PCL Viewer 關閉
    renderWindow->Finalize();  // 釋放 VTK 視窗資源
    renderWindow->GetInteractor()->TerminateApp();  // 強制關閉互動視窗
    // viewer.reset();  // 釋放記憶體
}

// 啟動 PCL Viewer
void runViewer(pcl::visualization::PCLVisualizer::Ptr viewer, vtkSmartPointer<vtkRenderWindow> renderWindow) {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    closeViewer(viewer, renderWindow);  // 確保關閉視窗
}

int main(int argc, char** argv) {
    // === 第一個 Viewer ===
    vtkSmartPointer<vtkRenderer> renderer1 = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow1 = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow1->AddRenderer(renderer1);
    
    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer(renderer1, renderWindow1, "Viewer 1"));
    viewer1->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    cloud1->width = 100;
    cloud1->height = 1;
    cloud1->points.resize(cloud1->width * cloud1->height);
    for (size_t i = 0; i < cloud1->points.size(); ++i) {
        cloud1->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud1->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud1->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    viewer1->addPointCloud<pcl::PointXYZ>(cloud1, "cloud1");

    // 啟動 Viewer 1
    runViewer(viewer1, renderWindow1);

    // === 第二個 Viewer ===
    vtkSmartPointer<vtkRenderer> renderer2 = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow2 = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);

    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "Viewer 2"));
    viewer2->setBackgroundColor(0.3, 0.3, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    cloud2->width = 100;
    cloud2->height = 1;
    cloud2->points.resize(cloud2->width * cloud2->height);
    for (size_t i = 0; i < cloud2->points.size(); ++i) {
        cloud2->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud2->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud2->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud2, 0, 255, 0);
    viewer2->addPointCloud<pcl::PointXYZ>(cloud2, green, "cloud2");

    // 啟動 Viewer 2
    runViewer(viewer2, renderWindow2);

    return 0;
}







// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <thread>
// #include <chrono>

// // 產生隨機點雲
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud() {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     cloud->width = 100;
//     cloud->height = 1;
//     cloud->points.resize(cloud->width * cloud->height);

//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud->points[i].r = 255; // 白色點雲
//         cloud->points[i].g = 255;
//         cloud->points[i].b = 255;
//     }
//     return cloud;
// }

// int main() {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = generatePointCloud();
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = generatePointCloud();

//     // 建立單一 Viewer
//     pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Multiple Viewports"));
//     viewer->setSize(800, 600);

//     // 定義兩個 Viewport
//     int v1(0), v2(1);
//     viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//     viewer->setBackgroundColor(0, 0, 0, v1);
//     viewer->addText("First Viewport", 10, 10, "v1 text", v1);
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud1);
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, rgb1, "cloud1", v1);

//     viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//     viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
//     viewer->addText("Second Viewport", 10, 10, "v2 text", v2);
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, rgb2, "cloud2", v2);

//     // 正常處理 Viewer 關閉
//     while (!viewer->wasStopped()) {
//         viewer->spinOnce(100);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     // 釋放 Viewer，確保視窗關閉
//     viewer->close();
//     viewer.reset();

//     return 0;
// }



// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/point_types.h>
// #include <thread>
// #include <chrono>

// void runViewer(pcl::visualization::PCLVisualizer *viewer) {
//     while (!viewer->wasStopped()) {
//         viewer->spinOnce(100);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//     viewer->close(); // 嘗試關閉視窗
// }

// int main(int argc, char** argv) {
//     // 第一個視窗
//     pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Viewer 1"));
//     viewer1->setBackgroundColor(0, 0, 0);
//     viewer1->addText("This is the first viewer", 10, 10, "text1");
//     viewer1->addCoordinateSystem(1.0);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud1->width = 100;
//     cloud1->height = 1;
//     cloud1->points.resize(cloud1->width * cloud1->height);
//     for (size_t i = 0; i < cloud1->points.size(); ++i) {
//         cloud1->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud1->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud1->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//     }
//     viewer1->addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud");

//     runViewer(viewer1.get());
//     // viewer1->getRenderWindow()->Finalize();  // 釋放 VTK 資源
//     viewer1.reset(); // 釋放記憶體
//     std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待 VTK 完全關閉

//     // 第二個視窗
//     pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Viewer 2"));
//     viewer2->setBackgroundColor(0.1, 0.1, 0.1);
//     viewer2->addText("This is the second viewer", 10, 10, "text2");
//     viewer2->addCoordinateSystem(1.0);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud2->width = 100;
//     cloud2->height = 1;
//     cloud2->points.resize(cloud2->width * cloud2->height);
//     for (size_t i = 0; i < cloud2->points.size(); ++i) {
//         cloud2->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud2->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud2->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//     }
//     viewer2->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud 2");

//     runViewer(viewer2.get());
//     // viewer2->getRenderWindow()->Finalize();  // 釋放 VTK 資源
//     viewer2.reset(); // 釋放記憶體

//     return 0;
// }










// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/point_types.h>
// #include <thread>
// #include <chrono> 

// void runViewer(pcl::visualization::PCLVisualizer &viewer) {
//     while (!viewer.wasStopped()) {
//         viewer.spinOnce(1);
//         // std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(2000));
        
//     }
//     viewer.close();
// }

// int main(int argc, char** argv) {
    
//     // 第一個視窗
//     pcl::visualization::PCLVisualizer viewer1("Viewer 1");
//     viewer1.setBackgroundColor(0, 0, 0);
//     viewer1.addText("This is the first viewer", 10, 10, "text1");
//     viewer1.addCoordinateSystem(1.0);
    
//     // 在第一個視窗中添加一些點雲或其他物件
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    
//     cloud1->width = 100;
//     cloud1->height = 1;
//     cloud1->points.resize(cloud1->width * cloud1->height);
//     for (size_t i = 0; i < cloud1->points.size(); ++i) {
//         cloud1->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud1->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//         cloud1->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//     }
//     viewer1.addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud");
//     runViewer(viewer1);
    

//     std::this_thread::sleep_for(std::chrono::seconds(5)); // 確保 VTK 完全釋放
//     // viewer1.close();


//     // // 啟動第一個視窗
//     // std::thread viewerThread1(runViewer, std::ref(viewer1));
//     // viewerThread1.detach(); // 分離線程以便主線程可以繼續

//     // // 等待第一個視窗關閉
//     // while (!viewer1.wasStopped()) {
//     //     std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
//     // }
//     for (int i=0;i<=100;i++)
//     {
//         cout<<"123\n";
//     }
//     // std::cin.get();
//     // // // 第二個視窗
//     // pcl::visualization::PCLVisualizer viewer2("Viewer 2");
//     // viewer2.setBackgroundColor(0.1, 0.1, 0.1);
//     // viewer2.addText("This is the second viewer", 10, 10, "text2");
//     // viewer2.addCoordinateSystem(1.0);

//     // // 在第二個視窗中添加一些點雲或其他物件
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//     // cloud2->width = 100;
//     // cloud2->height = 1;
//     // cloud2->points.resize(cloud2->width * cloud2->height);
//     // for (size_t i = 0; i < cloud2->points.size(); ++i) {
//     //     cloud2->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//     //     cloud2->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//     //     cloud2->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//     // }
//     // viewer2.addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud 2");

//     // // 啟動第二個視窗
//     // runViewer(viewer2); // 直接在主線程中運行第二個視窗

//     return 0;
// }
