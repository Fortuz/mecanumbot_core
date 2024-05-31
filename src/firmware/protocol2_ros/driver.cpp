#include "driver.h"

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
