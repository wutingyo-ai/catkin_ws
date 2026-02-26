#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <cstdlib>

#include <ros/callback_queue.h>

#include <sensor_msgs/Joy.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/remote_control.h>

#include "tm_msgs/AskSta.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ask_sta_test");
    ros::NodeHandle ask_demo;
    ros::ServiceClient client = ask_demo.serviceClient<tm_msgs::AskSta>("tm_driver/ask_sta");
    ros::Publisher joy_publisher_ = ask_demo.advertise<sensor_msgs::Joy>("/rviz_visual_tools_gui", 1);
    
    sensor_msgs::Joy msg;
    ROS_DEBUG_STREAM_NAMED("gui", "Next");
    msg.buttons.resize(9);
    msg.buttons[1] = 1;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    rviz_visual_tools::RemoteControl remote_control(ask_demo);
    visual_tools.loadRemoteControl();
    tm_msgs::AskSta srv;
    

    srv.request.subcmd = "00";
    srv.request.subdata = "";
    srv.request.wait_time = 1;
    

    
    // while (ros::ok())
    // {
      // if (remote_control.isWaiting())                             
      // {
      //   client.call(srv);
      //   if (srv.response.ok) {
    	//   remote_control.setReadyForNextStep();
      //   }    	
    //   }
    // }
  
  while (ros::ok())
  {
    if (client.call(srv))                             
    {
      if (srv.response.ok) {
    	  // ROS_INFO_STREAM("AskSta to robot: subcmd is " << srv.response.subcmd << ", subdata is " << srv.response.subdata); 
        // if (remote_control.isWaiting())                             
        // {
          joy_publisher_.publish(msg);
    	    remote_control.setFullAutonomous();
          remote_control.setReadyForNextStep();
          visual_tools.trigger();
          ros::spin();
          ROS_INFO_STREAM("AskSta to robot: subcmd is " << srv.response.subcmd << ", subdata is " << srv.response.subdata); 
          return 0;
        // }    	
      }    	
      else { 
    	  // ROS_WARN_STREAM("AskSta to robot , but response not yet ok ");
      }    	
    }
  }
  
  // if (client.call(srv))                             
  // {
  //   if (srv.response.ok) {
  //   	ROS_INFO_STREAM("AskSta to robot: subcmd is " << srv.response.subcmd << ", subdata is " << srv.response.subdata); 
  //   }    	
  //   else { 
  //   	ROS_WARN_STREAM("AskSta to robot , but response not yet ok ");
  //   }    	
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("Error AskSta to robot");
  //   return 1;
  // }

  ROS_INFO_STREAM_NAMED("AskSta", "shutdown.");  	
  return 0;

}