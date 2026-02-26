#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::visualization::CloudViewer viewer("PCL Viewer");

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    viewer.showCloud(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_viewer_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/pointcloud", 20, cloud_cb);
    ros::spin();
    return 0;
}
