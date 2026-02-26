#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <tm_msgs/SetPositions.h>
#include <tm_msgs/SendScript.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <robotiq_ft_sensor/ft_sensor.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <controller_manager_msgs/SwitchController.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>

#include <cstdlib>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <typeinfo>
#include <unistd.h>
#include <ctime>
#include <tf/tf.h>
#include <thread>
#include <stdio.h>
#include <vector>
#include <iostream>

geometry_msgs::Pose Current_Pose;
int arrive = 0;
size_t count = 0;

double fx = 0;
double fy = 0;
double fz = 0;
double mx = 0;
double my = 0;
double mz = 0;

// void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
// {
//     msg->joint
// }

void FTsensorcallback(const robotiq_ft_sensor::ft_sensor::ConstPtr& force_msg)
{
    fx = force_msg->Fx;
    fy = force_msg->Fy;
    fz = force_msg->Fz;
    mx = force_msg->Mx;
    my = force_msg->My;
    mz = force_msg->Mz;
}

/*void Switch_Controller(std::string start, std::string stop)
{
    ros::AsyncSpinner spinner(1);
    controller_manager_msgs::SwitchController Controller;
    Controller.request.start_controllers.push_back(start);
    Controller.request.stop_controllers.push_back(stop);
    Controller.request.strictness = 1;
    ros::service::call("/opt/ros/noetic/include/controller_manager_msgs/SwitchController",Controller);
}
*/

void gripper(ros::Publisher pub_gripper, int distance)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;
    gripper_command.rACT = 1;
    gripper_command.rATR = 0;
    gripper_command.rFR = 150;
    gripper_command.rGTO = 1;
    gripper_command.rPR = distance;
    gripper_command.rSP = 20;
    pub_gripper.publish(gripper_command);
    sleep(1);
}

void gripper_initialization(ros::Publisher pub_gripper)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;
    gripper_command.rACT = 0;
    gripper_command.rATR = 0;
    gripper_command.rFR = 0;
    gripper_command.rGTO = 0;
    gripper_command.rPR = 0;
    gripper_command.rSP = 0;
    pub_gripper.publish(gripper_command);
    sleep(1);
    gripper_command.rACT = 1;
    gripper_command.rATR = 0;
    gripper_command.rFR = 150;
    gripper_command.rGTO = 1;
    gripper_command.rPR = 0;
    gripper_command.rSP = 255;
    pub_gripper.publish(gripper_command);
    sleep(1);
}

// string Poi2Str(double P[])
// {
//     //std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)";
//     string strc_temp;
//     stringstream Ptrans[6];
//     for (str_count = 0; str_count < 6; str_count++)
//     {
//         Ptrans[str_count] << P[str_count];
//     }
//     string str[6] = {Ptrans[0].str(), Ptrans[1].str(), Ptrans[2].str(), Ptrans[3].str(), Ptrans[4].str(), Ptrans[5].str()};
//     strc_temp.append("PTP(\"CPP\",");
//     for (str_count = 0; str_count < 6; str_count++)
//     {
//         strc_temp.append(str[str_count]).append(",");
//     }
//     strc_temp.append("35,200,0,false)");
//     return strc_temp;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PickPlace");
    ros::NodeHandle P_Place;
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::ServiceClient set_position = P_Place.serviceClient<tm_msgs::SetPositions>("tm_driver/setpositions");
    ros::ServiceClient send_script = P_Place.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

    ros::Publisher marker_pub = P_Place.advertise<visualization_msgs::Marker>("visualization", 1);
    ros::Publisher chatter_pub = P_Place.advertise<std_msgs::String>("chatter_robot", 1000);
    moveit::planning_interface::MoveGroupInterface group("tmr_arm");
    group.setMaxVelocityScalingFactor(1);//Velocity
    group.setMaxAccelerationScalingFactor(1);//Acceleration
    group.setPlanningTime(0.1);//0.1
    moveit_msgs::Constraints constraints;
    moveit_msgs::JointConstraint joint_6_constraint;
    joint_6_constraint.joint_name = "joint_6";
    joint_6_constraint.position = 0;
    joint_6_constraint.tolerance_above = 1.91;
    joint_6_constraint.tolerance_below = 1.91;
    joint_6_constraint.weight = 1;
    constraints.joint_constraints.push_back(joint_6_constraint);
    group.setPathConstraints(constraints);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    geometry_msgs::Pose vision_point_1;
    vision_point_1.position.x = 0.518031;
    vision_point_1.position.y = -0.122313;
    vision_point_1.position.z = 0.463458; 
    vision_point_1.orientation.w = 0.001676;
    vision_point_1.orientation.x = 0.707097;
    vision_point_1.orientation.y = 0.707112;
    vision_point_1.orientation.z = 0.001656;
    group.execute(my_plan);
    group.setPoseTarget(vision_point_1);
    group.move();
    
    Current_Pose = group.getCurrentPose().pose;
    ROS_INFO("%f      %f     %f     %f     %f     %f    %f",Current_Pose.position.x,
     Current_Pose.position.y,Current_Pose.position.z,Current_Pose.orientation.w,Current_Pose.orientation.x,
     Current_Pose.orientation.y,Current_Pose.orientation.z);

    // ROS_INFO("%f",Current_Pose1);
    while(ros::ok());
}