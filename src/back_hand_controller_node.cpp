#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/Joy.h"

#define PROTOCOL_VERSION                2.0
#define DXL_ID                          20
#define LIDAR_DXL_ID                    1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/kudos_u2d2_B"     // "/dev/ttyUSB0"

#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT           102

#define ADDR_OPERATING_MODE             11
#define CURRENT_LIMIT                   38

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

#define DOWN_POSI 2075
#define UP_POSI 3230


#define PI 3.1415
#define ABS(X) ((X) < 0 ? -(X) : (X))
#define EPSILON 0.001


/********조이스틱 Axes & Butten*******/
/*#define*/ int AXES_LEFT_UPDOWN = 1;
//#define AXES_LEFT_RL -1

//#define AXES_RIGHT_UPDOWN -1
/*#define*/ int AXES_RIGHT_RL = 3;

/*#define*/ int BTTN_TRIANGLE = 2;
/*#define*/ int BTTN_SQUARE = 3;
/*#define*/ int BTTN_X = 0;
/*#define*/ int BTTN_O = 1;

/*#define*/ int BTTN_AXES_UPDOWN = 7;
/*#define*/ int BTTN_AXES_LEFTRIGHT = 6;

int Right_triger = 7;  //button (R2)
int Left_Triger = 6; //button (L2)

int Right_triger_button = 5; //5  (R1)
int Left_triger_button =4; //4  (L1)


bool posi_go = false;
bool gripper_posi_go = false;

int dxl20_initial_pose;
int present_position;

int Delta = 60;
int goal_position;
int gripper_goal_position;

int gripper_posi[2]={1890,745};

#define OPEN 1
#define CLOSE 0

int tele_onoff_g = 0;




#define DOWN_POSI 2075
#define UP_POSI 3230


#define PI 3.1415
#define ABS(X) ((X) < 0 ? -(X) : (X))
#define EPSILON 0.001

#define TORQUE_OFF 0  //operating mode
int operating_mode = -1;

int goal_updown = 1; //default : 1  : lidar down
bool is_moving = false;
int present_posi=0;
int goal_posi = 0;
int start=0;

double acc_time_area=1.5;
double dt = 0.01; //sec
double t = 0.0;

void updownCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if(msg->data != goal_updown){
    start = present_posi;
    goal_updown = msg->data;
    //is_moving = true;
    t = 0.0;
  }

}





void JoyCallback(const sensor_msgs::Joy::ConstPtr& joymsg)
{
  if(joymsg->axes[AXES_LEFT_UPDOWN]>0.2){
    goal_position = present_position - Delta;
    present_position = goal_position;
    posi_go = true;

  }
  else if(joymsg->axes[AXES_LEFT_UPDOWN]<-0.2){
    goal_position = present_position + Delta;
    present_position = goal_position;
    posi_go = true;
  }
  if(joymsg->buttons[Right_triger]>=0.9){
    gripper_goal_position = gripper_posi[CLOSE];
    gripper_posi_go = true;
    printf("aa");
  }
  else if(joymsg->buttons[Left_Triger]>=0.9){
    gripper_goal_position = gripper_posi[OPEN];
    gripper_posi_go = true;
  }

  if(joymsg->buttons[Right_triger_button]>=0.9){
    ROS_INFO("Right triger button : lidar updown");
    if(operating_mode == TORQUE_OFF){
      goal_updown = 1;
      start = present_posi;
      //is_moving = true;
      t = 0.0;
    }
  }
  else if(joymsg->buttons[Left_triger_button]>=0.9){
    if(operating_mode == TORQUE_OFF){
      ROS_INFO("Left triger button : lidar updown");
      goal_updown = 0;
      start = present_posi;
      //is_moving = true;
      t = 0.0;
    }
  }
}
void teleOnoffCallback(const std_msgs::Int8::ConstPtr& msg)
{
  tele_onoff_g = msg->data;
  if(tele_onoff_g == 4){
    operating_mode = TORQUE_OFF;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "back_hand_controller_node");
  ros::NodeHandle nh;

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
  }
  else{
    ROS_ERROR("Failed to open the port!");
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!");
  }
  else{
    ROS_ERROR("Failed to change the baudrate!\n");
    return 0;
  }



  //Set DXL 20 operating mode
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 5, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0){
    packetHandler->getRxPacketError(dxl_error);
  }
  else{
    ROS_INFO("Dynamixel 20 operating mode : current_based_position_mode");
  }


  //Set DXL 21 operating mode
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID+1, ADDR_OPERATING_MODE, 5, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0){
    packetHandler->getRxPacketError(dxl_error);
  }
  else{
    ROS_INFO("Dynamixel 11 operating mode : current_based_position_mode");
  }

  //Set dxl 11 current limit
 /* dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID+1, CURRENT_LIMIT, 000, &dxl_error); //2000 * 2.69 [mA]
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0){
    packetHandler->getRxPacketError(dxl_error);
  }
  else{
    ROS_INFO("Dynamixel 11 operating mode : current_based_position_mode");
  }*/





  // Enable Dynamixel Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0){
     packetHandler->getRxPacketError(dxl_error);
   }
   else{
     ROS_INFO("Dynamixel has been successfully connected");
   }

   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID+1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0){
     packetHandler->getRxPacketError(dxl_error);
   }
   else{
     ROS_INFO("Dynamixel has been successfully connected");
   }

   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, LIDAR_DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0){
     packetHandler->getRxPacketError(dxl_error);
   }
   else{
     ROS_INFO("Dynamixel has been successfully connected");
   }


   dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, 300, &dxl_error);

   dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID+1, ADDR_PRO_GOAL_CURRENT, 500, &dxl_error);


   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

   if (dxl_comm_result != COMM_SUCCESS)
   {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0)
   {
     packetHandler->getRxPacketError(dxl_error);
   }
   dxl20_initial_pose = dxl_present_position;
   present_position = dxl20_initial_pose;

   //lidar get_present_position
   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, LIDAR_DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0)
   {
     packetHandler->getRxPacketError(dxl_error);
   }
   present_posi = dxl_present_position;
   start = present_posi;


  ros::Subscriber sub = nh.subscribe("/joy", 1, JoyCallback);
  ros::Subscriber tele_onoff_sub = nh.subscribe("/teleop_onoff", 10, teleOnoffCallback);
  ros::Subscriber sub_lidar = nh.subscribe("/lidar_updown", 1000, updownCallback);



  ros::Rate loop_rate(50);
  while (ros::ok())
  {

    if((tele_onoff_g==3) && posi_go){
       dxl_comm_result = packetHandler->write4ByteTxOnly(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, (int)goal_position);
       if (dxl_comm_result != COMM_SUCCESS)
       {
         packetHandler->getTxRxResult(dxl_comm_result);
       }
       posi_go = false;
    }
    if((tele_onoff_g==3) && gripper_posi_go){

      dxl_comm_result = packetHandler->write4ByteTxOnly(portHandler, DXL_ID+1, ADDR_PRO_GOAL_POSITION, (int)gripper_goal_position);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      gripper_posi_go = false;

    }

    std_msgs::Int32 msg_;

    if(t < acc_time_area){
      if(goal_updown == 1){
        goal_posi = DOWN_POSI;
      }
      else if(goal_updown == 0){
        goal_posi = UP_POSI;

      }
      if(ABS(t-acc_time_area)<EPSILON){
        present_posi = goal_posi;
      }
      else present_posi = 0.5*(1-cos((PI)*(t/acc_time_area)))*(goal_posi-start) + start;
//       msg_.data = present_posi;
      t+=dt;

     }

     dxl_comm_result = packetHandler->write4ByteTxOnly(portHandler, LIDAR_DXL_ID, ADDR_PRO_GOAL_POSITION, (int)present_posi);
     if (dxl_comm_result != COMM_SUCCESS)
     {
       packetHandler->getTxRxResult(dxl_comm_result);
     }



    //dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
   // present_position = dxl_present_position;
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
