#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <cstdlib>

// #include "tm_msgs/SetEvent.h"
#include "tm_msgs/SendScript.h"

#include "geometry_msgs/Point.h"
// Robotiq導入
#include <robotiq_ft_sensor/ft_sensor.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <controller_manager_msgs/SwitchController.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <unistd.h>
#include <ros/console.h>
#include <typeinfo>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <iomanip>
#include <sstream>
#include <visualization_msgs/Marker.h>


using std::string;
using namespace std;


double pose_x = 0;
double pose_y = 0;
double pose_Rz = 0;
double pose_Ry = 0;

// double x_itera = 0;
// double y_itera = 0;



std::string bool_front_reverse;
std::string ACCC;

int arrive = 0;
size_t str_count = 0;


string Poi2Str(float []);


namespace choices{
  enum Option
  {
    choice_invalid,
    Front,
    Reverse
  };
}


choices::Option chooseOption(std::string input)
{
  if (input.compare("front") == 0 ) return choices::Option::Front;
  if (input.compare("reverse") == 0 ) return choices::Option::Reverse;
  return choices::Option::choice_invalid;
}

void front_reverse(const std_msgs::String::ConstPtr& bool_front_reverse)
{
  ACCC = bool_front_reverse->data.c_str();
  // ROS_INFO("Received : [%s]" , bool_front_reverse->data.c_str());
}


void python_point(const geometry_msgs::Point::ConstPtr& camera_point)
{
  pose_x = camera_point->x;
  pose_y = camera_point->y;
  pose_Rz = camera_point->z;
  // ROS_INFO("Received : [%f] [%f] [%f]", pose_x, pose_y, pose_Rz);
}

void Ry_angle(const std_msgs::Float64::ConstPtr& direct_ang) 
{
  pose_Ry = direct_ang->data;
  // ROS_INFO("Feature_angle : [%f]", pose_Ry);
}

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

void point_detect(double target_point)
{
  moveit::planning_interface::MoveGroupInterface move_group("tmr_arm");
  while (true)
      {
        if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_point)) < 0.00258 )  //高度在20(tcp),147.29(notcp)pose.position.z在0的時候是0.14729
        {
          arrive = arrive + 1;
          //std::cout << "goooooodPC1" << std::endl;
        }
        else if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_point)) > 0.00258 )
        {
          arrive = 0;
          //std::cout << "notyetPC1" << std::endl;
          //std::cout << move_group.getCurrentPose("link_6").pose.position.z << std::endl;
          
        }
        if (arrive > 3)
        {
          std::cout << "Success" << std::endl;
          arrive = 0;
          //gripper->goToPosition(0.008, 0.08, 90, true);
          //sleep(1);
          // strc = P2STR(PC0);
          break;
        }
      }
}

// 請記得先Reset、Active Robotiq夾爪 連線使用2f_gripper_control rtu, simplecontrol則是來active
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


int main(int argc, char **argv)
{   


    ros::init(argc, argv, "send_script_test");
    ros::NodeHandle demo_send_script;

    std::string Robo_TCP = "ChangeTCP(\"11081\")";
    std::string cmd = "ScriptExit()";

    static const std::string PLANNING_GROUP = "tmr_arm";
    ros::ServiceClient client = demo_send_script.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

    ros::Publisher pub_gripper = demo_send_script.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput",1);
    
    ros::Subscriber py_xy_point = demo_send_script.subscribe("robot_point_xy", 1000, python_point);
    ros::Subscriber py_front_reverse = demo_send_script.subscribe("front_reverse", 1000, front_reverse);
    ros::Subscriber py_Ry_ang = demo_send_script.subscribe("Ry_angle", 1000, Ry_angle);

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);  //对一组称为 “planning groups”的关节进行操作，并将其存储在名为JointModelGroup的对象中
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");

    tm_msgs::SendScript srv;
    tm_msgs::SendScript leave;
    tm_msgs::SendScript TCP;

    leave.request.id = "demo";
    leave.request.script = cmd;

    TCP.request.script = Robo_TCP;
    client.call(TCP);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    gripper_initialization(pub_gripper);
    int Place_count = 0;

    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
      ros::spinOnce();
      
      //sleep(4);
    


      double pose_x_new = pose_x;
      double pose_y_new = pose_y;
      double pose_Rz_new = pose_Rz;
      double pose_Ry_new = pose_Ry;
      std::cout << "pose_x_new:" << pose_x_new << endl;
      std::cout << "pose_y_new:" << pose_y_new << endl;
      std::cout << "pose_Rz_new:" << pose_Rz_new << endl;
      std::cout << "pose_Ry_new:" << pose_Ry_new << endl;

      
      //visual_tools.prompt("Press 'next' to ready1");
      // sleep(5);
      
      //  80  20378
      
      //點位1,目標物體上,姿態由topic訂閱
      double TCPts1[6] = {pose_x_new, pose_y_new, 100, -180, pose_Ry_new,  pose_Rz_new};
      string P1 = Poi2Str(TCPts1);
      srv.request.script = P1;
      client.call(srv);
      point_detect(0.23405);
      ROS_INFO("123");
      // 點位2,直下夾取目標物
      double TCPts2[6] = {pose_x_new, pose_y_new, 29.25, -180, pose_Ry_new, pose_Rz_new};
      string P2 = Poi2Str(TCPts2);
      srv.request.script = P2;
      client.call(srv);
      point_detect(0.15260);
      gripper(pub_gripper, 212);
      sleep(3);
      

      // 點位3,將物體抬起
      double TCPts3[6] = {pose_x_new, pose_y_new, 120, -180, pose_Ry_new, pose_Rz_new};
      string P3 = Poi2Str(TCPts3);
      srv.request.script = P3;
      client.call(srv);
      point_detect(0.24379);

      visual_tools.prompt("Press 'next' next");

      // srv.request.id = "demo";
      // if (client.call(srv))                             
      // {
      //     if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
      //     else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
      // }
      // else
      // {
      //     ROS_ERROR_STREAM("Error send script to robot");
      //     return 1;
      // }


      if ((chooseOption(ACCC) == choices::Option::Reverse) && (pose_Ry_new == 45))
      {
        //gripper11081 ok!
        double place_Rx1 = -10.88;
        double place_Ry1 = -482.91;
        //Place2
        //x:-10.88
        //y:-462.91
        //Place3
        //x:-10.88
        //y:-442.91
        //Place4
        //x:-10.88
        //y:-422.91
        //Place5
        //x:-10.88
        //y: -402.91


        double Rev_placeRx1 = -179.44;
        double Rev_placeRy1 = -45;
        double Rev_placeRz1 = -0.84;

        // double iter_Rx_ = 0;
        double iter_Ry1 = 20;

        //place_Rx += Place_count*iter_Rx;
        place_Ry1 += Place_count*iter_Ry1;

        double up_place1[6] = {place_Rx1, place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev1 = Poi2Str(up_place1);
        srv.request.script = place_Rev1;
        client.call(srv);
        point_detect(0.24379);

        double place_down1[6] = {place_Rx1, place_Ry1, 37.83, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev2 = Poi2Str(place_down1);
        srv.request.script = place_Rev2;
        client.call(srv);
        point_detect(0.16077);
        gripper(pub_gripper, 200);
        sleep(2);

        double place_ok1[6] = {place_Rx1, place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev3 = Poi2Str(place_ok1);
        srv.request.script = place_Rev3;
        client.call(srv);
        point_detect(0.24379);
      }
      
      else if ((chooseOption(ACCC) == choices::Option::Reverse) && (pose_Ry_new == -45))
      {
        //OK
        //gripper11081
        //Place1       
        double place_Rx2 = -11.11;
        double place_Ry2 = -482.63;
        //Place2
        //x:-11.11
        //y:-462.63
        //Place3
        //x:-11.11
        //y:-442.63
        //Place4
        //x:-11.11
        //y:-422.63
        //Place5
        //x:-11.11
        //y: -402.63

        double Rev_placeRx2 = 175.28;
        double Rev_placeRy2 = 45.00;
        double Rev_placeRz2 = 175.97;

        // double iter_Rx_1 = 0;
        double iter_Ry2 = 20;

        //place_Rx1 += Place_count*iter_Rx1;
        place_Ry2 += Place_count*iter_Ry2;

        //點位4,於治具上,並轉至放料姿態
        double up_place2[6] = {place_Rx2, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev4 = Poi2Str(up_place2);
        srv.request.script = place_Rev4;
        client.call(srv);
        point_detect(0.24379);

        //點位5,直下至治具,並放開夾爪
        double place_down2[6] = {place_Rx2, place_Ry2, 37.83, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev5 = Poi2Str(place_down2);
        srv.request.script = place_Rev5;
        client.call(srv);
        point_detect(0.16077);
        gripper(pub_gripper, 200);
        sleep(2);

        //點位6,將手臂抬起,最後回到Vision中
        double place_ok2[6] = {place_Rx2, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev6 = Poi2Str(place_ok2);
        srv.request.script = place_Rev6;
        client.call(srv);
        point_detect(0.24379);
      }


      
      else if ((chooseOption(ACCC) == choices::Option::Front) &&  (pose_Ry_new == -45))
      {
      //   ROS_INFO("situation1");
      //   //ok!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //   //gripper11081
      //   //Place1       
        double place_Fx1 = -10.23;
        double place_Fy1 = -482.98;
        //Place2
        //x:-10.23
        //y:-462.98
        //Place3
        //x:-10.23
        //y:-442.98
        //Place4
        //x:-10.23
        //y:-422.98
        //Place5
        //x:-10.23
        //y: -402.98

        double Fnt_placeFx1 = -179.44;
        double Fnt_placeFy1 = 40.00;
        double Fnt_placeFz1 = 1.37;

        // double iter_Fx = 0;
        double iter_Fy1 = 20.00;

        // place_Fx += Place_count*iter_Fx;
        place_Fy1 += Place_count*iter_Fy1;

        double up_place3[6] = {place_Fx1, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt1 = Poi2Str(up_place3);
        srv.request.script = place_Fnt1;
        client.call(srv);
        point_detect(0.24379);

        double place_down3[6] = {place_Fx1, place_Fy1, 37.83, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt2 = Poi2Str(place_down3);
        srv.request.script = place_Fnt2;
        client.call(srv);
        point_detect(0.17103);
        gripper(pub_gripper, 200);
        sleep(2);

        double place_ok3[6] = {place_Fx1, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt3 = Poi2Str(place_ok3);
        srv.request.script = place_Fnt3;
        client.call(srv);
        point_detect(0.25320);

      }
      
      else if ((chooseOption(ACCC) == choices::Option::Front) && (pose_Ry_new == 45))
      { 
        
        // gripper11081 OK!
        // Place1       
        double place_Fx2 = -10.27;
        double place_Fy2 = -482.43;
        //Place2
        //x:-10.27
        //y:-462.43
        //Place3
        //x:-10.27
        //y:-442.43
        //Place4
        //x:-9.46
        //y:-422.44
        //Place5
        //x:-9.46
        //y:-402.44
              
        double Fnt_placeFx2 = -179.44;
        double Fnt_placeFy2 = -40;
        double Fnt_placeFz2 = 179.13;

        // double iter_Fx = 0;
        double iter_Fy2 = 20.00;

        // place_Fx += Place_count*iter_Fx;
        place_Fy2 += Place_count*iter_Fy2;


        double up_place4[6] = {place_Fx2, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt4 = Poi2Str(up_place4);
        srv.request.script = place_Fnt4;
        client.call(srv);
        point_detect(0.24379);

        double place_down4[6] = {place_Fx2, place_Fy2, 37.83, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt5 = Poi2Str(place_down4);
        srv.request.script = place_Fnt5;
        client.call(srv);
        point_detect(0.17103);
        gripper(pub_gripper, 200);
        sleep(2);


        double place_ok4[6] = {place_Fx2, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt6 = Poi2Str(place_ok4);
        srv.request.script = place_Fnt6;
        client.call(srv);
        point_detect(0.25320);
      }
      
      // ROS_INFO("false");

    // 夾取五次條件
      Place_count += 1;
    // 退出（？
      if (Place_count == 5)
        break;
    
      gripper(pub_gripper, 150);

      

      leave.request.script = cmd;
      client.call(leave);
      // ROS_INFO(" [%d] ",Place_count);
      // loop_rate.sleep();

      visual_tools.prompt("Press 'next' to ready1");
      point_detect(0.24999);

      }
      
    // srv.request.script = P2;
    // client.call(srv);
    // point_detect(0.2564);
    // gripper(pub_gripper, 212);
    // sleep(3);

    // srv.request.script = P1;
    // client.call(srv);
    // point_detect(0.3);


    //visual_tools.prompt("Press 'next' next");
    
    

    //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");

    }
    // return 0;

