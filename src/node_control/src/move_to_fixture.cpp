#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

#include <sstream>
#include <cstdlib>
#include <vector>

// #include "tm_msgs/SetEvent.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/FeedbackState.h"
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

//請在class內調用回調物件
class Listener
{ 
  private: 
    bool IOsta;
  public:
    double pose_x;
    double pose_y;
    double pose_Rz;
    double pose_Ry;

    bool getIOsta() const; //要定義為const不可更改
    std::string ACCC;
    vector <float> place_xy;
    vector<float> refill_position;

    void front_reverse(const std_msgs::String::ConstPtr& bool_front_reverse);//正反面消息
    void python_point(const geometry_msgs::Point::ConstPtr& camera_point);
    void Ry_angle(const std_msgs::Float64::ConstPtr& direct_ang);
    void Point_place(const std_msgs::Float32MultiArray::ConstPtr& xy_pts); 
    void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg);
    void refill_callback(const std_msgs::Float32MultiArray::ConstPtr& failed_position);  //revise
    void IOcheck();
};


// double pose_x;
// double pose_y;
// double pose_Rz;
// double pose_Ry;

// vector <float> place_xy;

// std::string bool_front_reverse;
// std::string ACCC;

int arrive = 0; //到達點位之程度標準 大於3才算到達
size_t str_count = 0;


string Poi2Str(float []); //確認z點位
string Poi2StrPTP(float []);//確認相機姿態

namespace choices{ //正反面選擇定義
  enum Option
  {
    choice_invalid,
    Front,
    Reverse
  };
}


choices::Option chooseOption(std::string input)//正反面選擇定義
{
  if (input.compare("front") == 0 ) return choices::Option::Front;
  if (input.compare("reverse") == 0 ) return choices::Option::Reverse;
  return choices::Option::choice_invalid;
}

void Listener::front_reverse(const std_msgs::String::ConstPtr& bool_front_reverse) //正反面信息回調函數
{
  ACCC = bool_front_reverse->data.c_str();
  // ROS_INFO("Received : [%s]" , bool_front_reverse->data.c_str());
}


void Listener::python_point(const geometry_msgs::Point::ConstPtr& camera_point)//膠體點位資訊
{
  pose_x = camera_point->x;
  pose_y = camera_point->y;
  pose_Rz = camera_point->z;
  // ROS_INFO("Received : [%f] [%f] [%f]", pose_x, pose_y, pose_Rz);
}

void Listener::Ry_angle(const std_msgs::Float64::ConstPtr& direct_ang) //手臂取放傾角資訊
{
  pose_Ry = direct_ang->data;
  // ROS_INFO("Feature_angle : [%f]", pose_Ry);
}

//標定治具訊息的訂閱
void Listener::Point_place(const std_msgs::Float32MultiArray::ConstPtr& xy_pts)
{
  if (!xy_pts || xy_pts->data.empty()) {
    ROS_WARN("Received empty or null array");
    return;
  }

  place_xy.resize(xy_pts->data.size());//初始化一個名為 place_xy 的 vector,大小與輸入消息的資料大小相同

  for (int i = 0; i < place_xy.size(); ++i) { //遍歷輸入消息的資料,將每個元素賦值給 place_xy 中對應的元素。
    place_xy[i] = xy_pts->data[i];
  }

  for (int i = 0; i < place_xy.size(); ++i) {
    ROS_INFO("I heard place_xy %d: %f\n", i, place_xy[i]);
  }

}

void Listener::refill_callback(const std_msgs::Float32MultiArray::ConstPtr &detected_position_msg) /*revise*/
{
  if (!detected_position_msg || detected_position_msg->data.empty()) 
    {
      ROS_WARN("Received empty or null array");
      return;
    }

    refill_position.resize(detected_position_msg->data.size());//初始化

    for (int i = 0; i < refill_position.size(); ++i) 
    { //遍歷輸入消息的資料
      refill_position[i] = detected_position_msg->data[i];
    }

    for (int i = 0; i < refill_position.size(); ++i) 
    {
      //ROS_INFO("refill position of %d: %f\n", i, refill_position[i]);
    }
}

//發布線運動指令
string Poi2Str(double P[])
{
  string strc_temp;
  stringstream Ptrans[6];//stringstream 是 C++ 標準庫中的一個類,可以用於字串和數值之間的轉換。
  for (str_count = 0; str_count < 6; str_count++)
  {
      Ptrans[str_count] << P[str_count];//這裡使用了 << 運算符,它是 stringstream 類的輸出運算符,可以將數值寫入到字串中。
  }
  string str[6] = {Ptrans[0].str(), Ptrans[1].str(), Ptrans[2].str(), Ptrans[3].str(), Ptrans[4].str(), Ptrans[5].str()};//分別是x y z rx ry rz
  strc_temp.append("Line(\"CPP\",");
  for (str_count = 0; str_count < 6; str_count++)
  {
      strc_temp.append(str[str_count]).append(",");//整合字串
  }
  strc_temp.append("35,200,0,false)");
  return strc_temp;
}
//發布點到點運動指令
string Poi2StrPTP(double P[])
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


//若是要將下列兩定義合併,需要將rviz的link6之設定改為您的TCP

//此定義是確認TCP運動下其rviz界面中法蘭面的z資訊位置,故每次到點位時要確認法蘭面資訊的z
void point_detect(double target_point)
{
  moveit::planning_interface::MoveGroupInterface move_group("tmr_arm");
  while (true)
      { double point_offset=0.00258;
        if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_point)) < point_offset )  //高度在20(tcp),147.29(notcp)pose.position.z在0的時候是0.14729
        {                                                                                                  //0.0258原誤差
          arrive = arrive + 1;//到達點位之程度標準 大於3才算到達
        }
        else if (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_point)) > point_offset )
        {
          arrive = 0;
        }
        if (arrive > 3)
        {
          std::cout << "Success" << std::endl;
          arrive = 0;
          break;
        }
      }
}

//此定義是確認tcp運動下其rviz界面中法蘭面的x,y,z資訊位置,因為運動是下TCP位置,所以適用於固定的拍照點位
void pose_detect(double target_pose_x, double target_pose_y, double target_pose_z)
{
  moveit::planning_interface::MoveGroupInterface move_group("tmr_arm");
  while (true)
      { double point_offset=0.00258;
        if (
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.x) - abs(target_pose_x)) < point_offset )&&
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.y) - abs(target_pose_y)) < point_offset )&&
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_pose_z)) < point_offset )
           )
        {                                                                                                 
          arrive = arrive + 1;
        }
        else if (
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.x) - abs(target_pose_x)) < point_offset )&&
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.y) - abs(target_pose_y)) < point_offset )&&
        (abs(abs(move_group.getCurrentPose("link_6").pose.position.z) - abs(target_pose_z)) < point_offset )
                )
        {
          arrive = 0;
        }
        if (arrive > 3)
        {
          std::cout << "Success" << std::endl;
          arrive = 0;
          break;
        }
      }
}

void Listener::TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  IOsta = msg->cb_digital_input[0];//DI 達明FeedbackState的訊息
  // ROS_INFO("pin=(%d)", msg->cb_digital_input[0]);
}

bool Listener::getIOsta() const //須為const
{
    return IOsta;
}

void Listener::IOcheck()
{
    while (ros::ok())
    {
        if (getIOsta())
        {
            break;
        }
        ros::Duration(0.1).sleep(); // 每隔0.1秒等待一次
        ros::spinOnce(); // 讓 ROS 程式處理回呼函式
    }
}


// 請記得先Reset、Active Robotiq夾爪 連線使用2f_gripper_control rtu, simplecontrol則是來active
void gripper(ros::Publisher pub_gripper, int distance)
{
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;
  gripper_command.rACT = 1; //是否啟動夾爪
  gripper_command.rATR = 0;
  gripper_command.rFR = 150;
  gripper_command.rGTO = 1;
  gripper_command.rPR = distance; //*夾爪行程*
  gripper_command.rSP = 20;
  pub_gripper.publish(gripper_command);
  sleep(1);
}

void gripper_initialization(ros::Publisher pub_gripper)//自動初始化夾爪
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
  
  Listener listener;
  ros::init(argc, argv, "send_script_test");
  ros::NodeHandle demo_send_script;

  std::string Robo_TCP = "ChangeTCP(\"11081\")"; //改變為TCP坐標系 名稱為:11081
  std::string cmd = "ScriptExit()";
  
  static const std::string PLANNING_GROUP = "tmr_arm";
  ros::ServiceClient client = demo_send_script.serviceClient<tm_msgs::SendScript>("tm_driver/send_script"); //發送script服務

  ros::Publisher pub_gripper = demo_send_script.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput",1);
  
  ros::Subscriber py_xy_point = demo_send_script.subscribe("robot_point_xy", 10, &Listener::python_point, &listener); //&listener參數 確認類別
  ros::Subscriber py_front_reverse = demo_send_script.subscribe("front_reverse", 10, &Listener::front_reverse, &listener);
  ros::Subscriber py_Ry_ang = demo_send_script.subscribe("Ry_angle", 10, &Listener::Ry_angle, &listener);
  ros::Subscriber tar_place = demo_send_script.subscribe("Place_xy", 100, &Listener::Point_place, &listener);
  ros::Subscriber refill_subscriber = demo_send_script.subscribe("refill", 100, &Listener::refill_callback, &listener); //revise
  ros::Subscriber sub = demo_send_script.subscribe("feedback_states", 100, &Listener::TMmsgCallback, &listener);

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);  //对一组称为 “planning groups”的关节进行操作，并将其存储在名为JointModelGroup的对象中
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base");

  tm_msgs::SendScript srv;   //宣告變數 發送運動消息
  tm_msgs::SendScript leave; //宣告變數 離開listen node
  tm_msgs::SendScript TCP;   //宣告變數  轉換TCP

  leave.request.id = "demo";
  leave.request.script = cmd;

  TCP.request.script = Robo_TCP;
  client.call(TCP);

  ros::AsyncSpinner spinner(1); //多線程使用 通訊同步要用
  spinner.start();

  gripper_initialization(pub_gripper);

  //使用兩變數做Pallet判讀
  int Place_count = 0;
  int Pallet_count = 0;

  //使用兩變數夾取Pallet位置
  int pick_count_row = 0;
  int pick_count_colmn = 0;
  //使用變數放置Pallet位置
  int put_count = 0;//轉盤有無放10個之變數

  ros::Rate loop_rate(1000);

  //此while包含著兩段式運動的全部功能,若未來有心力,也可以寫成兩個定義來簡化整段程式
  while (ros::ok())//總程式迴圈
  {
    //一定會從散料區拍攝姿態開始

    
    pose_detect(0.00374, -0.55735, 0.25000);//檢查有無到法蘭面座標拍攝的位置
    ros::Duration(1.5).sleep(); //同步訊息之用
    
    //散料膠體依4種姿態排列至轉接治具 正反與開口上下組合
    while(ros::ok()) //////////////////////////////////////////////////放置之轉接製具之運動//////////////////////////////////     //第一階段運動 //
    {
      ros::spinOnce();
      
      //sleep(4);
      
      std::cout << "pose_x:" << listener.pose_x << endl;
      std::cout << "pose_y:" << listener.pose_y << endl;
      std::cout << "pose_Rz:" << listener.pose_Rz << endl;
      std::cout << "pose_Ry:" << listener.pose_Ry << endl;
      std::cout << "front_reverse:" << listener.ACCC << endl;
      
      //visual_tools.prompt("Press 'next' to ready1");
      gripper(pub_gripper, 180);//發布指夾行程為180
      //點位1,目標物體上,姿態由topic訂閱
      double TCPts1[6] = {listener.pose_x, listener.pose_y+0.6092, 100, -180.00, listener.pose_Ry,  listener.pose_Rz}; //100為TCP'11081'高度100mm
      string P1 = Poi2Str(TCPts1);
      srv.request.script = P1; // 定義命令
      client.call(srv); //呼叫請求點位的命令
      point_detect(0.22359); //法蘭面z的值 0.22359公尺
      // 0.23405
      ROS_INFO("123");
      // 點位2,直下夾取目標物
      // z:29.25mm point_detect(0.15260);
      // z:31mm point_detect(0.15546);
      // z:28.7mm point_detect(0.15315);
      double TCPts2[6] = {listener.pose_x, listener.pose_y+0.6092, 29, -180.00, listener.pose_Ry, listener.pose_Rz};
      string P2 = Poi2Str(TCPts2);
      srv.request.script = P2;
      client.call(srv);
      point_detect(0.15248);
      
      gripper(pub_gripper, 211.7);
      sleep(2); //同步夾爪運動
      
      // visual_tools.prompt("Press 'next' next");

      // 點位3,將物體抬起
      double TCPts3[6] = {listener.pose_x, listener.pose_y+0.6092, 140, -180.00, listener.pose_Ry, listener.pose_Rz};
      string P3 = Poi2Str(TCPts3);
      srv.request.script = P3;
      client.call(srv);
      point_detect(0.26443);

      // visual_tools.prompt("Press 'next' next");



      if ((chooseOption(listener.ACCC) == choices::Option::Reverse) && (listener.pose_Ry == 45))//第3次
      {
        //OK4 但不好校正
        //gripper11081 //第一個插槽點位參數資訊
        double place_Rx1 = 56.35;
        double place_Ry1 = -482.95;
        
        double Rev_placeRx1 = -180;
        double Rev_placeRy1 = -45;
        double Rev_placeRz1 = 0;

        //Pallet的x向量有微小誤差
        // double errx = 0.12;
        double errx = 0.3;

        //Pallet迭代量
        double iter_Rx = 13;
        double iter_Ry = 20;

        errx = Place_count*errx;
        place_Rx1 += Pallet_count*iter_Rx; //迭帶量位置
        place_Ry1 += Place_count*iter_Ry; //迭帶量位置

        double up_place1[6] = {place_Rx1+errx , place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1}; //到第一個插槽上面
        string place_Rev1 = Poi2Str(up_place1);
        srv.request.script = place_Rev1;
        client.call(srv);
        point_detect(0.24379);

        double med_pt[6] = {place_Rx1+errx , place_Ry1, 42, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1}; //離插槽很近之上方點位
        string mediapt = Poi2Str(med_pt);
        srv.request.script = mediapt;
        client.call(srv);
        point_detect(0.16549);

        // double place_down1[6] = {place_Rx1+errx , place_Ry1, 34.8, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1}; //插進插槽點位
        double place_down1[6] = {place_Rx1+errx , place_Ry1, 34.3, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1}; //插進插槽點位
        string place_Rev2 = Poi2Str(place_down1);
        srv.request.script = place_Rev2;
        client.call(srv);
        point_detect(0.15830);
        gripper(pub_gripper, 200); //開爪 不可太多
        sleep(2); //同步

        double place_ok1[6] = {place_Rx1+errx, place_Ry1, 120, Rev_placeRx1, Rev_placeRy1, Rev_placeRz1};
        string place_Rev3 = Poi2Str(place_ok1);
        srv.request.script = place_Rev3;
        client.call(srv);
        point_detect(0.24349);//回到初始place_ok1點位 
      }
      
      else if ((chooseOption(listener.ACCC) == choices::Option::Reverse) && (listener.pose_Ry == -45))//第2次
      {
        //OK 1
        //gripper11081 
        // 0424 有問題  
        double place_Rx2 = 57.00;
        double place_Ry2 = -483.20;

        double Rev_placeRx2 = -180;
        double Rev_placeRy2 = 45.00;
        double Rev_placeRz2 = 180;

        //Pallet的x向量有微小誤差
        double errx = 0.3;

        //Pallet迭代量
        double iter_Rx = 13;
        double iter_Ry = 20;

        errx = Place_count*errx;
        place_Rx2 += Pallet_count*iter_Rx;
        place_Ry2 += Place_count*iter_Ry;

        //點位4,於治具上,並轉至放料姿態
        double up_place2[6] = {place_Rx2+errx, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev4 = Poi2Str(up_place2);
        srv.request.script = place_Rev4;
        client.call(srv);
        point_detect(0.24379);

        double med_pt[6] = {place_Rx2+errx , place_Ry2, 42, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string mediapt = Poi2Str(med_pt);
        srv.request.script = mediapt;
        client.call(srv);
        point_detect(0.16549);

        //點位5,直下至治具,並放開夾爪
        // double place_down2[6] = {place_Rx2+errx, place_Ry2, 34.8, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        double place_down2[6] = {place_Rx2+errx, place_Ry2, 34.3, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev5 = Poi2Str(place_down2);
        srv.request.script = place_Rev5;
        client.call(srv);
        point_detect(0.15830);
        gripper(pub_gripper, 200);
        sleep(2);

        //點位6,將手臂抬起,最後回到Vision中
        double place_ok2[6] = {place_Rx2+errx, place_Ry2, 120, Rev_placeRx2, Rev_placeRy2, Rev_placeRz2};
        string place_Rev6 = Poi2Str(place_ok2);
        srv.request.script = place_Rev6;
        client.call(srv);
        point_detect(0.24379);
      }


      
      else if ((chooseOption(listener.ACCC) == choices::Option::Front) && (listener.pose_Ry == -45)) //第1次
      {
        ROS_INFO("situation1");
      //OK 3
      //gripper11081
      //Place1       
        double place_Fx1 = 56.30;
        double place_Fy1 = -482.95;

        double Fnt_placeFx1 = -180.00;
        double Fnt_placeFy1 = 45;
        double Fnt_placeFz1 = 0;

        //Pallet的x向量有微小誤差
        double errx = 0.3;

        //Pallet迭代量
        double iter_Fx = 13.00;
        double iter_Fy = 20.00;

        errx = Place_count*errx;
        place_Fx1 += Pallet_count*iter_Fx;
        place_Fy1 += Place_count*iter_Fy;

        double up_place3[6] = {place_Fx1+errx, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt1 = Poi2Str(up_place3);
        srv.request.script = place_Fnt1;
        client.call(srv);
        point_detect(0.24379);

        double med_pt[6] = {place_Fx1+errx , place_Fy1, 42, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string mediapt = Poi2Str(med_pt);
        srv.request.script = mediapt;
        client.call(srv);
        point_detect(0.16549);

        // double place_down3[6] = {place_Fx1+errx, place_Fy1, 34.8, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        double place_down3[6] = {place_Fx1+errx, place_Fy1, 34.3, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt2 = Poi2Str(place_down3);
        srv.request.script = place_Fnt2;
        client.call(srv);
        point_detect(0.15829);
        gripper(pub_gripper, 200);
        sleep(2);

        double place_ok3[6] = {place_Fx1+errx, place_Fy1, 120, Fnt_placeFx1, Fnt_placeFy1, Fnt_placeFz1};
        string place_Fnt3 = Poi2Str(place_ok3);
        srv.request.script = place_Fnt3;
        client.call(srv);
        point_detect(0.24349);

      }
      
      else if ((chooseOption(listener.ACCC) == choices::Option::Front) && (listener.pose_Ry == 45))//第4次
      { 
        //OK 2
        // gripper11081 
        // Place1       
        double place_Fx2 = 56.85;
        double place_Fy2 = -482.90;
              
        double Fnt_placeFx2 = -180;
        double Fnt_placeFy2 = -45;
        double Fnt_placeFz2 = 180;

        //Pallet的x向量有微小誤差
        double errx = 0.3;

        //Pallet迭代量        
        double iter_Fx = 13.00;
        double iter_Fy = 20.00;

        errx = Place_count*errx;
        place_Fx2 += Pallet_count*iter_Fx;
        place_Fy2 += Place_count*iter_Fy;


        double up_place4[6] = {place_Fx2+errx, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt4 = Poi2Str(up_place4);
        srv.request.script = place_Fnt4;
        client.call(srv);
        point_detect(0.24379);

        double med_pt[6] = {place_Fx2 +errx, place_Fy2, 42, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string mediapt = Poi2Str(med_pt);
        srv.request.script = mediapt;
        client.call(srv);
        point_detect(0.16549);

        // double place_down4[6] = {place_Fx2+errx, place_Fy2, 34.8, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        double place_down4[6] = {place_Fx2+errx, place_Fy2, 34.3, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt5 = Poi2Str(place_down4);
        srv.request.script = place_Fnt5;
        client.call(srv);
        point_detect(0.15829);
        gripper(pub_gripper, 200);
        sleep(2);


        double place_ok4[6] = {place_Fx2+errx, place_Fy2, 120, Fnt_placeFx2, Fnt_placeFy2, Fnt_placeFz2};
        string place_Fnt6 = Poi2Str(place_ok4);
        srv.request.script = place_Fnt6;
        client.call(srv);
        point_detect(0.24349);
      }
      
      
      // ROS_INFO("false");

      // 夾取五次
      Place_count += 1;//直行數值+1
      // Pallet結束並到插入端子治具步驟
      if (Pallet_count == 1 && Place_count == 5 ) //pallet count==1是第2列  Place_count == 5是第5行 表示結束任務
      { 
        Pallet_count = 0;
        Place_count = 0;
        break; //轉接製具運動跳出
      }
      
      else if (Place_count == 5)
      {
        Pallet_count = 1;
        Place_count = 0;
      }
      
    
      gripper(pub_gripper, 150); //張開一點點夾爪


      leave.request.script = cmd;
      client.call(leave); //退出listen node再去拍攝散料膠體
      ROS_INFO(" [%d] ",Place_count);
      loop_rate.sleep();

      // visual_tools.prompt("Press 'next' to ready1");

      //抵達拍照點位後讓程式停止1.5秒來確保筆電與TM的Listen有同步
      pose_detect(0.00370, -0.55735, 0.24999);
      ros::Duration(1.5).sleep();
    }
      
    ROS_INFO("did it");
    //先退出Listen結束第一階段運動
    leave.request.script = cmd;
    client.call(leave); //退出listen node

    ROS_INFO("error1"); 
   

    visual_tools.prompt("Press 'next' next");

    //一樣確認抵達拍照點位,先拍照
    // pose_detect(0.40283, -0.51050, 0.49997);

    ////////////////////////////////////////////////////轉接治具膠體插入端子治具//////////////////////////////////////////////////   //第二階段運動 //
    while (ros::ok())
    { 
      // gripper(pub_gripper, 180);//發布指夾行程為180 revise
      //先睡一下確保後續有直接進listen
      ROS_INFO("error2");
      // 要讀轉盤訊號再啟用
      // listener.IOcheck(); //DO0 是high才做插件
      ros::Duration(1).sleep(); //同步IO
      
      //先提供取放料參數與data

      //取料位置姿態
      //記得寫pick_x的累計誤差 從右到左 20條
      double errx = 0.16;
      errx = pick_count_row*errx;

      double pick_x = 56.80;
      double pick_y = -483.10;
      double pick_Rx = 180;
      double pick_Ry = 0;
      double pick_Rz = 180;

      // if (put_count == 0)
      // {
      //   pick_Rz = 0;
      // }
      // else
      // {
      //   pick_Rz = 180;
      // }
      
      //Pallet迭代參數
      double pick_interval_x = 13;
      double pick_interval_y = 20;
      pick_x += pick_count_colmn*pick_interval_x;
      pick_y += pick_count_row*pick_interval_y;

      //先執行取料行為
      //先到取料點上方
      double Point1[6] = {pick_x+errx, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
      ROS_INFO("error3");
      string pick_P1 = Poi2StrPTP(Point1); //要用PTP以防碰撞
      srv.request.script = pick_P1;
      client.call(srv);
      ROS_INFO("error");
      point_detect(0.27465);  
      ROS_INFO("error4");
      //取料
      double Point2[6] = {pick_x+errx, pick_y, 35.6, pick_Rx, pick_Ry, pick_Rz};//35.5測試高度 36.03原本高度
      string pick_P2 = Poi2Str(Point2);
      srv.request.script = pick_P2;
      client.call(srv);
      point_detect(0.21010);//0.21067
      gripper(pub_gripper, 211.6);
      sleep(2);

      //回到上方
      double Point3[6] = {pick_x+errx, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
      string pick_P3 = Poi2StrPTP(Point3);
      srv.request.script= pick_P3;
      client.call(srv);
      point_detect(0.27465);

      //先退出Listen
      leave.request.script = cmd;
      client.call(leave);

      //一樣確認抵達拍照點位,先拍照
      // pose_detect(0.48112, -0.52050, 0.25028); 
      pose_detect(0.48165, -0.52073, 0.25028);
      //先睡一下確保後續有直接進listen
      ros::Duration(1).sleep();
      //回調陣列,接收訂閱訊息
      ros::spinOnce();

      //標定治具點位設定與取值,註：前10個為x值,後10個為y值
      // vector <float> place_loc = listener.place_xy;
      // 0.6092
      float put_x = listener.place_xy.at(put_count); //x1 x2 x3 x4... x10   使用at方法訪問元素較安全方便
      float put_y = listener.place_xy.at(put_count+10)+0.3294; //y1 y2 y3 y4 ... y10 插槽位置y軸補償0.6092
      double put_Rx = 180;
      double put_Ry = 0;
      double put_Rz = 180;

      // if (put_count == 0)
      // {
      //   put_Rz = 0;
      // }
      // else
      // {
      //   put_Rz = 180;
      // }

      // visual_tools.prompt("Press 'next' next");
      //然後至轉盤端子治具上
      double Point4[6] = {put_x, put_y, 50, put_Rx, put_Ry, put_Rz}; //轉盤端子置具正上方
      string place_P1 = Poi2StrPTP(Point4);
      srv.request.script= place_P1;
      client.call(srv);
      point_detect(0.22454);

      // visual_tools.prompt("Press 'next' next");

      //放料
      double Point5[6] = {put_x, put_y, 13.56, put_Rx, put_Ry, put_Rz};
      string  place_P2 = Poi2Str(Point5);
      srv.request.script= place_P2;
      client.call(srv);
      point_detect(0.18820);
      gripper(pub_gripper, 200);
      sleep(2);

      //再回到上方
      double Point6[6] = {put_x, put_y, 50, put_Rx, put_Ry, put_Rz};
      string  place_P3 = Poi2StrPTP(Point6);
      srv.request.script= place_P3;
      client.call(srv);
      point_detect(0.22454);
      gripper(pub_gripper, 180);
      //pick_count_colmn & pick_count_colmn 為 pallet放置兩變數
      pick_count_row += 1;
      //放置計數
      put_count += 1;
      
      // if (listener.refill_position.at(put_count-1)==0 && put_count!=0)//revise 檢測條件
      // {
      //   put_count-=1;
      // }
      
      
      //放完10次即跳出,並回到迴圈最開始
      if (put_count == 10)
      {
        put_count = 0;
        pick_count_row = 0;
        pick_count_colmn = 0;
        visual_tools.prompt("Press 'next' next");
        leave.request.script = cmd;
        client.call(leave);
        break;
      }
      //換排
      else if (pick_count_row == 5)
      {
        pick_count_colmn = 1;
        pick_count_row = 0;
      }
      //回到迴圈起始點

    }

  //補料運動
  /* while (ros::ok())
  {
    //一樣確認抵達拍照點位,先拍照
    pose_detect(0.48210, -0.51713, 0.25000);
    //先睡一下確保後續有直接進listen
    ros::Duration(1).sleep();
    //回調陣列,接收訂閱訊息
    ROS_INFO("Before spinOnce: %ld", listener.refill_position.size());
    ros::spinOnce();
    ROS_INFO("After spinOnce: %ld", listener.refill_position.size());
    float detection=0.0;
    ROS_INFO("Start Refill");

    for(int i=0;i<=9;i++)
    {
      detection=listener.refill_position.at(i);
      ROS_INFO("loop Received refill position : [%d] [%f]\n", i,detection);

    }
    
    ROS_INFO("finish Refill");



  } */
  



  }

}

