// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/SendScript.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connect");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::ConnectTM>("tm_diver/connect_tm");
  ros::ServiceClient send_script = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::ConnectTM srv;
  tm_msgs::SendScript leave;



  std::string cmd = "ScriptExit()";

  //Request
  srv.request.server = srv.request.TMSVR;
  srv.request.reconnect = true;
  srv.request.timeout = 0;
  srv.request.timeval = 1;
// while (ros::ok())
// {
  if (client.call(srv))                             
    {
      if (srv.response.ok) 
      {
        ROS_INFO_STREAM("ConnectTM to robot");
        leave.request.script = cmd;
        send_script.call(leave);
        // break;
      }
      else ROS_WARN_STREAM("ConnectTM to robot , but response not yet ok ");
    }
  else
    {
      ROS_ERROR_STREAM("Error ConnectTM to robot");
      return 1;
    }
// }

  //ROS_INFO_STREAM_NAMED("ConnectTM", "shutdown.");  	
  return 0;
}