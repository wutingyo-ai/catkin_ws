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
        {                                                                                                  //0.0258原誤差
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

    //使用兩變數做Pallet判讀
    int Place_count = 0;
    int Pallet_count = 0;

    //使用兩變數夾取Pallet位置
    int pick_count_row = 0;
    int pick_count_colmn = 0;
    //使用變數放置Pallet位置
    int put_count = 0;

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
      point_detect(0.22359);
      // 0.23405
      ROS_INFO("123");
      // 點位2,直下夾取目標物
      // z:29.25mm point_detect(0.15260);
      // z:31mm point_detect(0.15546);
      // z:28.7mm point_detect(0.15315);
      double TCPts2[6] = {pose_x_new, pose_y_new, 28.7, -180, pose_Ry_new, pose_Rz_new};
      string P2 = Poi2Str(TCPts2);
      srv.request.script = P2;
      client.call(srv);
      point_detect(0.15220);
      
      gripper(pub_gripper, 212);
      sleep(2);
      
      // visual_tools.prompt("Press 'next' next");

      // 點位3,將物體抬起
      double TCPts3[6] = {pose_x_new, pose_y_new, 140, -180, pose_Ry_new, pose_Rz_new};
      string P3 = Poi2Str(TCPts3);
      srv.request.script = P3;
      client.call(srv);
      point_detect(0.26443);

      visual_tools.prompt("Press 'next' next");



      if ((chooseOption(ACCC) == choices::Option::Reverse) && (pose_Ry_new == 45))
      {
        //gripper11081 
        double place_Rx1 = -24.40;
        double place_Ry1 = -467.55;
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


        double Rev_placeRx1 = -180;
        double Rev_placeRy1 = -45;
        double Rev_placeRz1 = 0;

        // double iter_Rx = 13;
        // double iter_Ry = 20;

        // place_Rx1 += Pallet_count*iter_Rx;
        // place_Ry1 += Place_count*iter_Ry;

        double up_place1[6] = {place_Rx1, place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev1 = Poi2Str(up_place1);
        srv.request.script = place_Rev1;
        client.call(srv);
        point_detect(0.24379);

        double place_down1[6] = {place_Rx1, place_Ry1, 32.20, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev2 = Poi2Str(place_down1);
        srv.request.script = place_Rev2;
        client.call(srv);
        point_detect(0.15570);
        gripper(pub_gripper, 200);
        sleep(2);

        double place_ok1[6] = {place_Rx1, place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev3 = Poi2Str(place_ok1);
        srv.request.script = place_Rev3;
        client.call(srv);
        point_detect(0.24349);
      }
      
      else if ((chooseOption(ACCC) == choices::Option::Reverse) && (pose_Ry_new == -45))
      {
        //OK
        //gripper11081
        //Place1       
        double place_Rx2 = -24.15;
        double place_Ry2 = -467.35;
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

        double Rev_placeRx2 = -180;
        double Rev_placeRy2 = 45.00;
        double Rev_placeRz2 = 180;

        // double iter_Rx = 13;
        // double iter_Ry = 20;

        // place_Rx2 += Pallet_count*iter_Rx;
        // place_Ry2 += Place_count*iter_Ry;

        //點位4,於治具上,並轉至放料姿態
        double up_place2[6] = {place_Rx2, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev4 = Poi2Str(up_place2);
        srv.request.script = place_Rev4;
        client.call(srv);
        point_detect(0.24379);

        //點位5,直下至治具,並放開夾爪
        double place_down2[6] = {place_Rx2, place_Ry2, 32.20, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev5 = Poi2Str(place_down2);
        srv.request.script = place_Rev5;
        client.call(srv);
        point_detect(0.15570);
        gripper(pub_gripper, 200);
        sleep(2);

        //點位6,將手臂抬起,最後回到Vision中
        double place_ok2[6] = {place_Rx2, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev6 = Poi2Str(place_ok2);
        srv.request.script = place_Rev6;
        client.call(srv);
        point_detect(0.24379);
      }


      
      else if ((chooseOption(ACCC) == choices::Option::Front) && (pose_Ry_new == -45))
      {
        ROS_INFO("situation1");
      //   
      //   //gripper11081
      //   //Place1       
        double place_Fx1 = -23.85;
        double place_Fy1 = -467.10;


        double Fnt_placeFx1 = -180.00;
        double Fnt_placeFy1 = 45;
        double Fnt_placeFz1 = 0;

        // double iter_Fx = 13.00;
        // double iter_Fy = 20.00;

        // place_Fx1 += Pallet_count*iter_Fx;
        // place_Fy1 += Place_count*iter_Fy;

        double up_place3[6] = {place_Fx1, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt1 = Poi2Str(up_place3);
        srv.request.script = place_Fnt1;
        client.call(srv);
        point_detect(0.24379);

        double place_down3[6] = {place_Fx1, place_Fy1, 32.20, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt2 = Poi2Str(place_down3);
        srv.request.script = place_Fnt2;
        client.call(srv);
        point_detect(0.15570);
        gripper(pub_gripper, 200);
        sleep(2);

        double place_ok3[6] = {place_Fx1, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt3 = Poi2Str(place_ok3);
        srv.request.script = place_Fnt3;
        client.call(srv);
        point_detect(0.24349);

      }
      
      else if ((chooseOption(ACCC) == choices::Option::Front) && (pose_Ry_new == 45))
      { 
        
        // gripper11081 需要重校
        // Place1       
        double place_Fx2 = -24.10;
        double place_Fy2 = -466.75;
              
        double Fnt_placeFx2 = -180;
        double Fnt_placeFy2 = -45;
        double Fnt_placeFz2 = 180;

        // double iter_Fx = 13.00;
        // double iter_Fy = 20.00;

        // place_Fx2 += Pallet_count*iter_Fx;
        // place_Fy2 += Place_count*iter_Fy;


        double up_place4[6] = {place_Fx2, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt4 = Poi2Str(up_place4);
        srv.request.script = place_Fnt4;
        client.call(srv);
        point_detect(0.24379);

        double place_down4[6] = {place_Fx2, place_Fy2, 32.20, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt5 = Poi2Str(place_down4);
        srv.request.script = place_Fnt5;
        client.call(srv);
        point_detect(0.15570);
        gripper(pub_gripper, 200);
        sleep(2);


        double place_ok4[6] = {place_Fx2, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt6 = Poi2Str(place_ok4);
        srv.request.script = place_Fnt6;
        client.call(srv);
        point_detect(0.24349);
      }
      
      // ROS_INFO("false");

    // 夾取五次
    //   Place_count += 1;
    // Pallet結束並到插入端子治具步驟
    //   if (Pallet_count == 1 && Place_count == 5 )
    //   { 
    //     Pallet_count = 0;
    //     Place_count = 0;
    //     break;
    //   }
      
    //   else if (Place_count == 5)
    //   {
    //     Pallet_count = 1;
    //     Place_count = 0;
    //   }
          
    
      gripper(pub_gripper, 150);


      leave.request.script = cmd;
      client.call(leave);
      ROS_INFO(" [%d] ",Place_count);
      loop_rate.sleep();

      visual_tools.prompt("Press 'next' to ready1");
      point_detect(0.24999);
      

      }
      
    ROS_INFO("did it");

    visual_tools.prompt("Press 'next' next");
    
    //轉接治具膠體插入端子治具
    // while (ros::ok())
    // { 

    //   double pick_x = 58.1;
    //   double pick_y = -483.2;
    //   double pick_Rx = 180;
    //   double pick_Ry = 0;
    //   double pick_Rz = 180;

    //   double pick_interval_x = 13;
    //   double pick_interval_y = 20;

    //   pick_x += pick_count_colmn*pick_interval_x;
    //   pick_y += pick_count_row*pick_interval_y;


      
    //   double put_x = 243.60;
    //   double put_y = -444.10;
    //   double put_Rx = 180;
    //   double put_Ry = 0;
    //   double put_Rz = 180;

    //   double put_interval = 2.25;

    //   put_x += put_count*put_interval;

    //   double Point1[6] = {pick_x, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
    //   string pick_P1 = Poi2Str(Point1);
    //   srv.request.script= pick_P1;
    //   client.call(srv);
    //   point_detect(0.27465);  

    //   double Point2[6] = {pick_x, pick_y, 37.2, pick_Rx, pick_Ry, pick_Rz};
    //   string pick_P2 = Poi2Str(Point2);
    //   srv.request.script= pick_P2;
    //   client.call(srv);
    //   point_detect(0.21184);
    //   gripper(pub_gripper, 212);
    //   sleep(2);

    //   double Point3[6] = {pick_x, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
    //   string pick_P3 = Poi2Str(Point3);
    //   srv.request.script= pick_P3;
    //   client.call(srv);
    //   point_detect(0.27465);

      

    //   double Point4[6] = {put_x, put_y, 100, put_Rx, put_Ry, put_Rz};
    //   string place_P1 = Poi2Str(Point4);
    //   srv.request.script= place_P1;
    //   client.call(srv);
    //   point_detect(0.27465);

    //   visual_tools.prompt("Press 'next' next");

    //   double Point5[6] = {put_x, put_y, 44.75, put_Rx, put_Ry, put_Rz};
    //   string  place_P2 = Poi2Str(Point5);
    //   srv.request.script= place_P2;
    //   client.call(srv);
    //   point_detect(0.21940);
    //   gripper(pub_gripper, 200);
    //   sleep(2);

    //   double Point6[6] = {put_x, put_y, 100, put_Rx, put_Ry, put_Rz};
    //   string  place_P3 = Poi2Str(Point6);
    //   srv.request.script= place_P3;
    //   client.call(srv);
    //   point_detect(0.27465);


    //   pick_count_row += 1;
    //   put_count += 1;

    //   if (put_count == 10)
    //   {
    //     put_count = 0;
    //     pick_count_row = 0;
    //     pick_count_colmn = 0;
    //     break;
    //   }
      
    //   else if (pick_count_row == 5)
    //   {
    //     pick_count_colmn = 1;
    //     pick_count_row = 0;
    //   }

    // }
    
    return 0;
 

    }
    // return 0;