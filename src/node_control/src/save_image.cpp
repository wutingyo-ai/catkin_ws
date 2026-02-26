#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

cv::Mat image(320,480,CV_8UC3);
cv::Mat image2;
int encoding_to_mat_type(const std::string & encoding){
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  }else if (encoding =="8UC3"){
    return CV_8UC3;
  }
  else {
    std::cout<<"the unknow image type is "<<encoding<<std::endl;
    throw std::runtime_error("Unsupported encoding type");
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
    cv::Mat frame(msg->height, msg->width, encoding_to_mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    std::cout << "Width : " << frame.size().width << std::endl;
    std::cout << "Height: " << frame.size().height << std::endl;
    frame.copyTo(image);
    std::cout<<"after set this->image = frame";
    //cv::imwrite("/home/chen/save_image/puipui.jpg", image);
  }
  catch(std::runtime_error &exception){
    std::cout<<"there is a exception "<< exception.what()<< std::endl;
  }
}

// int show_image(){
//   while(true){
//     cv::resize(image,image,cv::Size(1600,900),0,0, cv::INTER_AREA);
    
//     cv::cvtColor(image,image2,cv::COLOR_BGR2GRAY);
//     cv::GaussianBlur(image2, image2, cv::Size(7, 7), 7, 7);
//     cv::threshold(image2,image2,100,255,CV_THRESH_BINARY);
//     cv::imshow("view",image2 );
//     cv::waitKey(30);
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  // cv::Mat frame;
  // cv::imshow("view",frame);

  ros::Subscriber sub = nh.subscribe("techman_image", 1000, imageCallback);

  // std::thread(show_image).detach();
    //spin時間段設定,確保能退出程序,並輸出圖片
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(3.0).sleep();
    spinner.stop();
    cv::imwrite("/home/chen/save_image/pick.jpg", image);


  //cv::destroyWindow("view");

}
