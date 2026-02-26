#include <ros/ros.h>
#include <std_msgs/String.h>

#include "tm_msgs/FeedbackState.h"

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
    if(msg->joint_pos.size() == 6)
    {
        ROS_INFO_STREAM("FeedbackState: joint pos = (" <<
                 msg->joint_pos[0] << ", " <<
                 msg->joint_pos[1] << ", " <<
                 msg->joint_pos[2] << ", " <<
                 msg->joint_pos[3] << ", " <<
                 msg->joint_pos[4] << ", " <<
                 msg->joint_pos[5] << ")");
    }
    else
    {
        ROS_ERROR_STREAM("Error FeedbackState callback");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FeedbackState");
    ros::NodeHandle get;
    ros::Subscriber sub = get.subscribe("feefback_states", 1000, TMmsgCallback);
    ros::spin();
    return 0;
}
    