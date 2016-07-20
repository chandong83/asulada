#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "dynamixel_sdk.h"                                   // Uses Dynamixel SDK library

// Control table address (Dynamixel EX series)
#define ADDR_EX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_EX_GOAL_POSITION           30
#define ADDR_EX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                       1               // Value for enabling the torque
#define TORQUE_DISABLE                      0               // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE          100             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE          4000            // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD         10              // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                     0x1b
#define SMALL_U_ASCII_VALUE                 0x75
#define SMALL_D_ASCII_VALUE                 0x64

namespace dynamixel_controller
{
class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Topic Subscriber
  ros::Subscriber position_sub_;
  // ROS Topic Publisher
  ros::Publisher position_pub_;
  //parameters
  bool is_debug_;
  std::string device_name_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  uint8_t dxl_id_;
  int32_t goal_position_;
  int32_t current_position_;

 public:
  DynamixelController();
  ~DynamixelController();
  void checkLoop(void);
  void closeDynamixel(void);

 private:
  bool initDynamixelController(void);
  bool shutdownDynamixelController(void);
  void positionCallback(const std_msgs::Int32::ConstPtr& position);
  bool setTorque(uint8_t id, bool onoff);
  bool readPosition(uint8_t id, int32_t &position);
  void writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value);
  void readDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length);
};
}

#endif // eof DYNAMIXEL_CONTROLLER_H
