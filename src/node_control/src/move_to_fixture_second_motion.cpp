// 2024 09 09 record
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
// TM Driver header
#include "tm_msgs/WriteItem.h"


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
    ROS_WARN("Received empty or null array for xy_pts");
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
      ROS_WARN("Received empty or null array for refill");
      return;
    }

    refill_position.resize(detected_position_msg->data.size());//初始化

    for (int i = 0; i < refill_position.size(); ++i) 
    { //遍歷輸入消息的資料
      refill_position[i] = detected_position_msg->data[i];
    }

    for (int i = 0; i < refill_position.size(); ++i) 
    {
      ROS_INFO("refill position of %d: %f\n", i, refill_position[i]);
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

//發布線運動絕對速度指令
string Poi2Str_absolute(double P[],int speed,int time,int trajectory_percent)
{
  string strc_temp;
  string change_to_string0;
  string change_to_string1;
  string change_to_string2;
  stringstream Ptrans[6];//stringstream 是 C++ 標準庫中的一個類,可以用於字串和數值之間的轉換。
  for (str_count = 0; str_count < 6; str_count++)
  {
      Ptrans[str_count] << P[str_count];//這裡使用了 << 運算符,它是 stringstream 類的輸出運算符,可以將數值寫入到字串中。
  }
  string str[6] = {Ptrans[0].str(), Ptrans[1].str(), Ptrans[2].str(), Ptrans[3].str(), Ptrans[4].str(), Ptrans[5].str()};//分別是x y z rx ry rz

  strc_temp.append("Line(\"CAP\",");

  for (str_count = 0; str_count < 6; str_count++)
  {
      strc_temp.append(str[str_count]).append(",");//整合字串
  }

  
  change_to_string0=to_string(speed);
  change_to_string1=to_string(time);
  change_to_string2=to_string(trajectory_percent);

  strc_temp.append(change_to_string0).append(",").append(change_to_string1).append(",").append(change_to_string2).append(",false)");
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



//發布線運動軌跡混成指令
string Poi2Str_trajectory(double P[],int speed,int time,int trajectory_percent)
{
  string strc_temp;
  string change_to_string0;
  string change_to_string1;
  string change_to_string2;
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

  
  change_to_string0=to_string(speed);
  change_to_string1=to_string(time);
  change_to_string2=to_string(trajectory_percent);

  strc_temp.append(change_to_string0).append(",").append(change_to_string1).append(",").append(change_to_string2).append(",false)");
  return strc_temp;
}

//發布點到點運動軌跡混成指令
string Poi2StrPTP_trajectory(double P[],int speed,int time,int trajectory_percent)
{
  string strc_temp;
  string change_to_string0;
  string change_to_string1;
  string change_to_string2;
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

  change_to_string0=to_string(speed);
  change_to_string1=to_string(time);
  change_to_string2=to_string(trajectory_percent);

  strc_temp.append(change_to_string0).append(",").append(change_to_string1).append(",").append(change_to_string2).append(",false)");
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
  ros::ServiceClient termial_fixture_client = demo_send_script.serviceClient<tm_msgs::WriteItem>("tm_driver/write_item"); //發送完成插端子指令信息給TM revise
  ros::Publisher pub_gripper = demo_send_script.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput",1);
  
  ros::Subscriber py_xy_point = demo_send_script.subscribe("robot_point_xy", 10, &Listener::python_point, &listener); //&listener參數 確認類別
  ros::Subscriber py_front_reverse = demo_send_script.subscribe("front_reverse", 10, &Listener::front_reverse, &listener);
  ros::Subscriber py_Ry_ang = demo_send_script.subscribe("Ry_angle", 10, &Listener::Ry_angle, &listener);
  ros::Subscriber tar_place = demo_send_script.subscribe("Place_xy", 100, &Listener::Point_place, &listener);
  ros::Subscriber refill_subscriber = demo_send_script.subscribe("refill", 100, &Listener::refill_callback, &listener); //revise
  // ros::Subscriber sub = demo_send_script.subscribe("feedback_states", 100, &Listener::TMmsgCallback, &listener);

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);  //对一组称为 “planning groups”的关节进行操作，并将其存储在名为JointModelGroup的对象中
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base");

  // double test[6]={0,1,2,3,4,5};//revise
  // string test_string=Poi2StrPTP_trajectory(test,50);//revise
  // ROS_INFO("script=%s",test_string.c_str());//revise
  // visual_tools.prompt("Press 'next' next");//revise

  tm_msgs::SendScript srv;   //宣告變數 發送運動消息
  tm_msgs::SendScript leave; //宣告變數 離開listen node
  tm_msgs::SendScript TCP;   //宣告變數  轉換TCP
  tm_msgs::WriteItem write_srv; //宣告變數 發送完成插料指令


  leave.request.id = "demo";
  leave.request.script = cmd;

  write_srv.request.id = "detect"; //ID不可以取名跟TMflow一樣之名稱 //revise
  write_srv.request.item = "g_complete_signal";//revise
  write_srv.request.value = "false";//revise
  termial_fixture_client.call(write_srv);//revise

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

  int pick_total_count=0;
  int put_count = 0;//轉盤有無放10個之變數 

  //使用變數放置Pallet位置 第二轉接治具
  int pick_count_row_second = 0;  //revise
  int pick_count_colmn_second = 0; //revise

  //軌跡混成百分比參數
  int trajectory_percent=100; //revise

  //速度設定
  int speed=100;
  int low_speed=15;
  int absolute_speed=133.33;

  ros::Rate loop_rate(1000);

  //此while包含著兩段式運動的全部功能,若未來有心力,也可以寫成兩個定義來簡化整段程式
  while (ros::ok())//總程式迴圈
  {
    ////////////////////////////////////////////////////轉接治具膠體插入端子治具//////////////////////////////////////////////////   //第二階段運動 //
    while (ros::ok())
    { 
      gripper(pub_gripper, 180);//發布指夾行程為180 revise
      //先睡一下確保後續有直接進listen
      ROS_INFO("STEP 2");
      // 要讀轉盤訊號再啟用
      // listener.IOcheck(); //DO0 是high才做插件
      ros::Duration(1).sleep(); //同步IO
      double listener_pose_y_err=0.0;





      ////////////////////////////////////////////////////////////////////第一轉接治具
      if(pick_total_count<10) // 取料次數多寡 revise
      // if(pick_total_count<=-1) // 取料次數多寡 revise
      {
          //放完10次即跳出,並回到迴圈最開始
        if (put_count == 10)
        {
          put_count = 0;

          pick_count_row = 0;
          pick_count_colmn = 0;
          
          pick_count_row_second = 0; //revise
          pick_count_colmn_second = 0; //revise

          write_srv.request.id = "detect"; //ID不可以取名跟TMflow一樣之名稱 //revise
          write_srv.request.item = "g_complete_signal";//revise
          write_srv.request.value = "true";//revise
          termial_fixture_client.call(write_srv);//revise
          //先睡一下確保後續有直接進listen
          ros::Duration(0.5).sleep(); //revise

          leave.request.script = cmd;
          client.call(leave);
          break;
        }
            
            
            
            //先提供取放料參數與data

          //取料位置姿態
          //記得寫pick_x的累計誤差 從右到左 20條
          double errx = 0.2;
          errx = pick_count_row*errx;

          double pick_x = 57.1; //2024 09 02 :double pick_x = 56.26; 2024/10/16 56.56
          double pick_y = -483.22; //2024 09 02: double pick_y = -482.72;
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
          // ROS_INFO("error3");
          string pick_P1 = Poi2StrPTP_trajectory(Point1,speed,200,trajectory_percent); //要用PTP以防碰撞
          srv.request.script = pick_P1;
          client.call(srv);
          ROS_INFO("Pick high");
          point_detect(0.27465);  
          ROS_INFO("Complete Pick high!!!");
          //取料
          double Point2[6] = {pick_x+errx, pick_y, 35.7, pick_Rx, pick_Ry, pick_Rz};//35.5測試高度 36.03原本高度
          string pick_P2 = Poi2Str_absolute(Point2,absolute_speed,200,trajectory_percent);
          srv.request.script = pick_P2;
          client.call(srv);
          point_detect(0.21020);//0.21067
          gripper(pub_gripper, 211.6);
          sleep(2);

          //取了第幾次料
          pick_total_count+=1; //revise 檢測取了幾次
          ROS_WARN("pick_total_count=%d;\n",pick_total_count);

          //回到上方
          double Point3[6] = {pick_x+errx, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
          string pick_P3 = Poi2StrPTP_trajectory(Point3,speed,200,trajectory_percent); 
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
          

            if (put_count!=0 )//revise 檢測條件
            {
              if (listener.refill_position.at(put_count-1)==0 )//revise 檢測條件
              {
                put_count-=1;
                ROS_WARN("Detect of refill potion,Position=%d;\n",put_count);
              }
            }

          


          //標定治具點位設定與取值,註：前10個為x值,後10個為y值
          // vector <float> place_loc = listener.place_xy;
          // 0.6092
          float put_x = listener.place_xy.at(put_count); //x1 x2 x3 x4... x10   使用at方法訪問元素較安全方便
          float put_y = listener.place_xy.at(put_count+10)+listener_pose_y_err; //y1 y2 y3 y4 ... y10 插槽位置y軸補償0.6092
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
          string place_P1 = Poi2Str_absolute(Point4,absolute_speed,200,trajectory_percent);
          srv.request.script= place_P1;
          client.call(srv);
          point_detect(0.22454);

          // visual_tools.prompt("Press 'next' next");

          //放料
          double Point5[6] = {put_x, put_y, 13.56, put_Rx, put_Ry, put_Rz}; // 2024/10/15 27.24
          string  place_P2 = Poi2Str_absolute(Point5,absolute_speed,200,trajectory_percent);
          srv.request.script= place_P2;
          client.call(srv);
          point_detect(0.18820); // 2024/10/14 0.20189
          gripper(pub_gripper, 200);
          sleep(2);

          //再回到上方
          double Point6[6] = {put_x, put_y, 50, put_Rx, put_Ry, put_Rz};
          string  place_P3 = Poi2StrPTP_trajectory(Point6,speed,200,trajectory_percent);
          srv.request.script= place_P3;
          client.call(srv);
          point_detect(0.22454);
          gripper(pub_gripper, 180);
          //pick_count_colmn & pick_count_colmn 為 pallet放置兩變數
          pick_count_row += 1;
          
          //放置計數
          put_count += 1;
          ROS_WARN("put_count=%d\n;",put_count);
          
          
          
          //換排
          if (pick_count_row == 5)
          {
            pick_count_colmn = 1;
            pick_count_row = 0;
          }
      }







      ////////////////////////////////////////////////////////////第二轉接治具
      // else  if (pick_total_count>=10 && pick_total_count<=20)
        else
       {
          ROS_WARN("The Second Fixture!!!");
        //放完10次即跳出,並回到迴圈最開始
          if (put_count == 10)
          {
            put_count = 0;

            pick_count_row= 0; //revise
            pick_count_colmn = 0; //revise

            pick_count_row_second = 0; //revise
            pick_count_colmn_second = 0; //revise

            write_srv.request.id = "010"; //ID不可以取名跟TMflow一樣之名稱 //revise
            write_srv.request.item = "g_complete_signal";//revise
            write_srv.request.value = "true";//revise
            termial_fixture_client.call(write_srv);//revise
            //先睡一下確保後續有直接進listen
            ros::Duration(0.5).sleep(); //revise

            leave.request.script = cmd;
            client.call(leave);
            break;
          }

          

         //先提供取放料參數與data

          //取料位置姿態
          //記得寫pick_x的累計誤差 從右到左 20條
          double errx = 0.2;
          errx = pick_count_row_second*errx;//revise

          double pick_x = -92.91;
          double pick_y = -482.23; // double pick_y = -482.23;
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
          pick_x += pick_count_colmn_second*pick_interval_x;//revise
          pick_y += pick_count_row_second*pick_interval_y;//revise

          //先執行取料行為
          //先到取料點上方
          double Point1[6] = {pick_x+errx, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
          // ROS_INFO("Pick high second");
          string pick_P1 = Poi2StrPTP_trajectory(Point1,speed,200,trajectory_percent); //要用PTP以防碰撞
          srv.request.script = pick_P1;
          client.call(srv);
          ROS_INFO("Pick high second");
          point_detect(0.27465);  
          ROS_INFO("Complete Pick high second !!");
          //取料
          double Point2[6] = {pick_x+errx, pick_y, 35.7, pick_Rx, pick_Ry, pick_Rz};//35.5測試高度 36.03原本高度
          string pick_P2 = Poi2Str_absolute(Point2,absolute_speed,200,trajectory_percent);
          srv.request.script = pick_P2;
          client.call(srv);
          point_detect(0.21020);//0.21067
          gripper(pub_gripper, 211.6);
          sleep(2);

          //取了第幾次料
          pick_total_count+=1; //revise 檢測取了幾次
          ROS_WARN("pick_total_count=%d\n;",pick_total_count);

          //回到上方
          double Point3[6] = {pick_x+errx, pick_y, 100, pick_Rx, pick_Ry, pick_Rz};
          string pick_P3 = Poi2StrPTP_trajectory(Point3,speed,200,trajectory_percent);
          srv.request.script= pick_P3;
          client.call(srv);
          point_detect(0.27465);
          ros::spinOnce();  
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
          // ROS_WARN("SpinOnce\n");
          
         
          if (put_count!=0 )//revise 檢測條件
            {
              if (listener.refill_position.at(put_count-1)==0 )//revise 檢測條件
              {
                put_count-=1;
                ROS_WARN("Detect of refill potion,Position=%d;\n",put_count);
              }
            }
        

          //標定治具點位設定與取值,註：前10個為x值,後10個為y值
          // vector <float> place_loc = listener.place_xy;
          // 0.6092
          float put_x = listener.place_xy.at(put_count); //x1 x2 x3 x4... x10   使用at方法訪問元素較安全方便
          float put_y = listener.place_xy.at(put_count+10)+listener_pose_y_err; //y1 y2 y3 y4 ... y10 插槽位置y軸補償0.6092
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
          string place_P1 = Poi2Str_absolute(Point4,absolute_speed,200,0);
          srv.request.script= place_P1;
          client.call(srv);
          point_detect(0.22454);

          // visual_tools.prompt("Press 'next' next");

          //放料
          double Point5[6] = {put_x, put_y, 13.56, put_Rx, put_Ry, put_Rz};
          string  place_P2 = Poi2Str_absolute(Point5,absolute_speed,200,trajectory_percent);
          srv.request.script= place_P2;
          client.call(srv);
          point_detect(0.18820);
          gripper(pub_gripper, 200);
          sleep(2);

          //再回到上方
          double Point6[6] = {put_x, put_y, 50, put_Rx, put_Ry, put_Rz};
          string  place_P3 = Poi2StrPTP_trajectory(Point6,speed,200,trajectory_percent);
          srv.request.script= place_P3;
          client.call(srv);
          point_detect(0.22454);
          gripper(pub_gripper, 180);
          //pick_count_colmn & pick_count_colmn 為 pallet放置兩變數
          pick_count_row_second += 1;//revise
          
          //放置計數
          put_count += 1;
          ROS_INFO("put_count=%d;\n",put_count);
          
          
          
          
          //換排
          if (pick_count_row_second == 5) //revise
          {
            pick_count_colmn_second = 1;
            pick_count_row_second = 0;
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

}

