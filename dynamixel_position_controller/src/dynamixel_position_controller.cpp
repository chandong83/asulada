/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Yoonseok Pyo */

#include "dynamixel_position_controller/dynamixel_position_controller.h"

using namespace dynamixel_controller;

DynamixelController::DynamixelController()
: nh_priv_("~"),
  goal_position_(0),
  current_position_(0),
  dxl_id_(1),
  device_name_(DEVICENAME),
  is_debug_(false)
{
  //Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.param("device_name", device_name_, device_name_);

  //Init target name
  ROS_ASSERT(initDynamixelController());
  position_sub_ = nh_.subscribe("/asulada_goal_position", 1, &DynamixelController::positionCallback, this);
  position_pub_ = nh_.advertise<std_msgs::Int32>("/asulada_current_position", 10);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize Packethandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if( portHandler_->openPort() )
  {
      ROS_INFO("Succeeded to open the port!");
  }
  else
  {
      ROS_ERROR("Failed to open the port!");
      shutdownDynamixelController();
  }

  // Set port baudrate
  if( portHandler_->setBaudRate(BAUDRATE) )
  {
      ROS_INFO("Succeeded to change the baudrate!");
  }
  else
  {
      ROS_ERROR("Failed to change the baudrate!");
      shutdownDynamixelController();
  }

  // Enable Dynamixel Torque
  setTorque(dxl_id_, true);
}

DynamixelController::~DynamixelController()
{
  ROS_ASSERT(shutdownDynamixelController());
}

bool DynamixelController::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_EX_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
      packetHandler_->printTxRxResult(dxl_comm_result);
  else if(dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
}

bool DynamixelController::readPosition(uint8_t id, int32_t &position)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t dxl_present_position = 0;

  dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_EX_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS)
  {
      packetHandler_->printTxRxResult(dxl_comm_result);
      return false;
  }
  else if(dxl_error != 0)
  {
      packetHandler_->printRxPacketError(dxl_error);
      return false;
  }

  position = dxl_present_position;

  return true;
}

void DynamixelController::writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!",id);
  }
}

void DynamixelController::readDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;

  int8_t  value8    = 0;
  int16_t value16   = 0;
  int32_t value32   = 0;


  if (length == 1)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value8, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value16, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value32, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) packetHandler_->printRxPacketError(dxl_error);

    if (length == 1)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value8);
    }
    else if (length == 2)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value16);
    }
    else if (length == 4)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value32);
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
  }
}

void DynamixelController::checkLoop(void)
{
  std_msgs::Int32 current_pos;

  writeDynamixelRegister(dxl_id_, ADDR_EX_GOAL_POSITION, 2, (int32_t)goal_position_);
  ROS_INFO("[ID] %u, [Goal Position] %d", dxl_id_, (int32_t)goal_position_);

  readPosition(dxl_id_, current_position_);
  current_pos.data = current_position_;
  position_pub_.publish(current_pos);
}

void DynamixelController::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(dxl_id_, false);
  // Close port
  portHandler_->closePort();
}

bool DynamixelController::initDynamixelController()
{
  ROS_INFO("dynamixel_controller_node : Init OK!");
  return true;
}

bool DynamixelController::shutdownDynamixelController()
{
  return true;
}

void DynamixelController::positionCallback(const std_msgs::Int32::ConstPtr& position)
{
  goal_position_ = position->data;
}

int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "dynamixel_position_controller_node");
  DynamixelController dc;
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    dc.checkLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  dc.closeDynamixel();

  return 0;
}
