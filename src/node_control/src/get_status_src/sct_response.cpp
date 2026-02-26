#include <ros/ros.h>
#include <std_msgs/String.h>

#include "tm_msgs/SctResponse.h"

void SctResponseCallback(const tm_msgs::SctResponse::ConstPtr& msg)
{
    std::string Callback_Print = "SctResponse callback: id is " + msg->id + ",script is " + msg->script;
    ROS_INFO_STREAM(Callback_Print);
    ROS_INFO_STREAM("SctResponse: id is = " << msg->id << ", script is " << msg->script);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SctResponse");
    ros::NodeHandle get;
    ros::Subscriber sub = get.subscribe("tm_driver/sct_response", 1000, SctResponseCallback);
    ros::spin();
    return 0;
}