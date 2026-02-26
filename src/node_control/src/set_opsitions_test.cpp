#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

#include "tm_msgs/SetPositions.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_positions_test");
    ros::NodeHandle positions_test;
    ros::ServiceClient client = positions_test.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    tm_msgs::SetPositions srv;

    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(1.58);
    srv.request.positions.push_back(0);
    srv.request.positions.push_back(1.58);
    srv.request.positions.push_back(0);
    srv.request.velocity = 0.4;//rad/s
    srv.request.acc_time = 0.2;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal  = false;

    if (client.call(srv))                             
    {
        if (srv.response.ok) ROS_INFO_STREAM("SetPositions to robot");
        else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
    }
    else
    {
        ROS_ERROR_STREAM("Error SetPositions to robot");
        return 1;
    }

  //ROS_INFO_STREAM_NAMED("SetPositions", "shutdown.");  	
    return 0;
}