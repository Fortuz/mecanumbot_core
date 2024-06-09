/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "mecanum_motor_driver.h"

/*********************************************************************************
 * Mecanum Motor Driver
 ********************************************************************************/
MecanumMotorDriver::MecanumMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION2),
  left_rear_wheel_id_(DXL_LEFT_REAR_ID), right_rear_wheel_id_(DXL_RIGHT_REAR_ID),
  left_front_wheel_id_(DXL_LEFT_FRONT_ID), right_front_wheel_id_(DXL_RIGHT_FRONT_ID)
{
}

MecanumMotorDriver::~MecanumMotorDriver()
{
  closeDynamixel();
}

bool MecanumMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(left_rear_wheel_id_, true);
  setTorque(right_rear_wheel_id_, true);
  setTorque(left_front_wheel_id_, true);
  setTorque(right_front_wheel_id_, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);

  return true;
}

bool MecanumMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MecanumMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_rear_wheel_id_, false);
  setTorque(right_rear_wheel_id_, false);
  setTorque(left_front_wheel_id_, false);
  setTorque(right_front_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool MecanumMotorDriver::controlMotor(int64_t left_rear_wheel_value, int64_t right_rear_wheel_value, int64_t left_front_wheel_value, int64_t right_front_wheel_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)&left_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)&right_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)&left_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)&right_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

/*********************************************************************************
 * Neck Motor Driver
 ********************************************************************************/
NeckMotorDriver::NeckMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION2),
  neck_yaw_id_(DXL_NECK_YAW_ID), neck_pitch_id_(DXL_NECK_PITCH_ID)
{
}

NeckMotorDriver::~NeckMotorDriver()
{
  closeDynamixel();
}

bool NeckMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(neck_yaw_id_, true);
  setTorque(neck_pitch_id_, true);

  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  return true;
}

bool NeckMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void NeckMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(neck_yaw_id_, false);
  setTorque(neck_pitch_id_, false);

  // Close port
  portHandler_->closePort();
}

bool NeckMotorDriver::controlMotor(int64_t neck_yaw_value, int64_t neck_pitch_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(neck_yaw_id_, (uint8_t*)&neck_yaw_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(neck_pitch_id_, (uint8_t*)&neck_pitch_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

/*********************************************************************************
 * Grabber Motor Driver
 ********************************************************************************/
 GrabberMotorDriver::GrabberMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION1),
  left_grabber_id_(DXL_LEFT_GRABBER_ID), right_grabber_id_(DXL_RIGHT_GRABBER_ID)
{
}

GrabberMotorDriver::~GrabberMotorDriver()
{
  closeDynamixel();
}

bool GrabberMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(left_grabber_id_, true);
  setTorque(right_grabber_id_, true);

  //groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  
  return true;
}

bool GrabberMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void GrabberMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_grabber_id_, false);
  setTorque(right_grabber_id_, false);

  // Close port
  portHandler_->closePort();
}

bool GrabberMotorDriver::controlMotor(int64_t left_grabber_value, int64_t right_grabber_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(left_grabber_id_, (uint8_t*)&left_grabber_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(right_grabber_id_, (uint8_t*)&right_grabber_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}
