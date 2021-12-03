#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/Joy.h"

#define PROTOCOL_VERSION                2.0
#define DXL_ID                          20
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_PRESENT_POSITION       132

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

int Right_triger = 7;  //button
int Left_Triger = 6; //button


bool posi_go = false;

int dxl20_initial_pose;
int present_position;

int Delta = 50;
int goal_position;

int tele_onoff_g = 0;


void JoyCallback(const sensor_msgs::Joy::ConstPtr& joymsg)
{
  if(joymsg->axes[AXES_LEFT_UPDOWN]>0.1){
    goal_position = present_position + Delta;
    present_position = goal_position;
    posi_go = true;

  }
  else if(joymsg->axes[AXES_LEFT_UPDOWN]<0.1){
    goal_position = present_position - Delta;
    present_position = goal_position;
    posi_go = true;

  }

}
void teleOnoffCallback(const std_msgs::Int8::ConstPtr& msg)
{
  tele_onoff_g = msg->data;

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


  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = nh.subscribe("/joy", 1, JoyCallback);
  ros::Subscriber tele_onoff_sub = nh.subscribe("/teleop_onoff", 10, teleOnoffCallback);




  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    if((tele_onoff_g==3) && posi_go){
       dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, (int)goal_position, &dxl_error);
       if (dxl_comm_result != COMM_SUCCESS)
       {
         packetHandler->getTxRxResult(dxl_comm_result);
       }
       else if (dxl_error != 0)
       {
         packetHandler->getRxPacketError(dxl_error);
       }
       posi_go = false;
    }


    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
