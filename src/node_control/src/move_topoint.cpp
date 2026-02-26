#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <unistd.h>
#include <ros/console.h>
#include <typeinfo>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

#include <actionlib/client/simple_action_client.h>

#include <std_msgs/String.h>
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/AskItem.h"
#include "tm_msgs/AskSta.h"
#include "tm_msgs/SendScript.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

using std::string;
using namespace std;

visualization_msgs::Marker points;

int arrive = 0;
size_t str_count = 0 ;
string strc;
string Poi2Str(float []);

string Poi2Str(double P[])
{
    string strc_temp;
    stringstream Ptrans[6];
    for (str_count = 0; str_count < 6; str_count++)
    {
        Ptrans[str_count] << P[str_count];
    }
    string str[6] = {Ptrans[0].str(), Ptrans[1].str(), Ptrans[2].str(), Ptrans[3].str(), Ptrans[4].str(), Ptrans[5].str()};
    strc_temp.append("PTP(\"CPP\",");
    for (str_count = 0; str_count < 6; str_count++)
    {
        strc_temp.append(str[str_count]).append(",");
    }
    strc_temp.append("35,200,0,false)");
    return strc_temp;
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");

    ros::NodeHandle node_handle;

    ros::ServiceClient set_position_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    ros::ServiceClient set_event = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
    ros::ServiceClient ask_sta = node_handle.serviceClient<tm_msgs::AskSta>("tm_driver/ask_sta");
    ros::ServiceClient client = node_handle.serviceClient<tm_msgs::AskItem>("tm_driver/ask_item");
    ros::ServiceClient send_script = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");


    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization", 1);
    ros::Publisher chatter_pub = node_handle.advertise<std_msgs::String>("chatter_robot", 1000);

    tm_msgs::SetPositions position_srv;
    tm_msgs::SetEvent event;
    tm_msgs::AskSta sta;
    tm_msgs::SendScript sendscript;

    ros::AsyncSpinner spinner{1};
    spinner.start();

    static const std::string PLANNING_GROUP = "tmr_arm";
    moveit::planning_interface::MoveGroupInterface move_group{PLANNING_GROUP};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    set_position_client.call(position_srv);

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    visual_tools.deleteAllMarkers();
    visual_tools.prompt("Press 'next' to ready1");
    while (ros::ok())
    {
        marker_pub.publish(points);
    
        ros::spinOnce();
        visual_tools.loadRemoteControl();
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();

        ROS_INFO_NAMED("turorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
        ROS_INFO_NAMED("turtorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
        ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    
        while (ros::ok())
        {
            visual_tools.prompt("Press 'next' to Ready1");

            double TCPts[6] = {389.44, 78.99, 353.02, -177.91, -4.27, 18.39};
            strc = Poi2Str(TCPts);
            sendscript.request.id = "demo";
            sendscript.request.script = strc;

            if (send_script.call(sendscript))
            {
                if (sendscript.response.ok) ROS_INFO_STREAM("Sent script to robot");
                else ROS_WARN_STREAM("Sent script to robot, but response not yet ok ");
            }
            else ROS_ERROR_STREAM("Error send script to robot");

            while (ros::ok())
            {
                if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(0.53240)) < 0.002 )
                {
                    arrive += 1;
                }
                else if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(0.53240)) > 0.002 )
                {
                    arrive = 0;
                }
                else if (arrive > 3)
                {
                    std::cout << "Success!" << std::endl;
                    arrive = 0;
       
                    sleep(1);
                    break;
                }
                
            }
            
        }

    }

}



