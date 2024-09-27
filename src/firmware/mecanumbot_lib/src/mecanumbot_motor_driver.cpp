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

#include "../include/mecanumbot_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882; 

/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_LEFT_FRONT = 2; // ID of front left motor
const uint8_t DXL_MOTOR_ID_RIGHT_FRONT = 1; // ID of front right motor
const uint8_t DXL_MOTOR_ID_LEFT_BACK = 4; // ID of rear left motor
const uint8_t DXL_MOTOR_ID_RIGHT_BACK = 3; // ID of rear right motor
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);



Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: left_front_wheel_id_(DXL_MOTOR_ID_LEFT_FRONT),
  right_front_wheel_id_(DXL_MOTOR_ID_RIGHT_FRONT),
  left_back_wheel_id_(DXL_MOTOR_ID_LEFT_BACK),
  right_back_wheel_id_(DXL_MOTOR_ID_RIGHT_BACK),
  torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_write_param.id_count = 4;
  sync_write_param.xel[LEFT_FRONT].id = left_front_wheel_id_;
  sync_write_param.xel[RIGHT_FRONT].id = right_front_wheel_id_;
  sync_write_param.xel[LEFT_BACK].id = left_back_wheel_id_;
  sync_write_param.xel[RIGHT_BACK].id = right_back_wheel_id_;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_FRONT].id = left_front_wheel_id_;
  sync_read_param.xel[RIGHT_FRONT].id = right_front_wheel_id_;
  sync_read_param.xel[LEFT_BACK].id = left_back_wheel_id_;
  sync_read_param.xel[RIGHT_BACK].id = right_back_wheel_id_;

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

Dynamixel2Arduino& Turtlebot3MotorDriver::getDxl()
{
  return dxl;
}

bool Turtlebot3MotorDriver::is_connected()
{
  return (dxl.ping(DXL_MOTOR_ID_LEFT_FRONT) == true && dxl.ping(DXL_MOTOR_ID_RIGHT_FRONT) == true) && (dxl.ping(DXL_MOTOR_ID_LEFT_BACK) == true && dxl.ping(DXL_MOTOR_ID_RIGHT_BACK) == true);
}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[LEFT_FRONT].data[0] = onoff;
  sync_write_param.xel[RIGHT_FRONT].data[0] = onoff;
  sync_write_param.xel[LEFT_BACK].data[0] = onoff;
  sync_write_param.xel[RIGHT_BACK].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool Turtlebot3MotorDriver::get_torque()
{
  if(dxl.readControlTableItem(TORQUE_ENABLE, left_front_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, right_front_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, left_back_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, right_back_wheel_id_) == true){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool Turtlebot3MotorDriver::read_present_position(int32_t &left_front_value, int32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_front_value, read_result.xel[LEFT_FRONT].data, read_result.xel[LEFT_FRONT].length);
    memcpy(&right_front_value, read_result.xel[RIGHT_FRONT].data, read_result.xel[RIGHT_FRONT].length);
    memcpy(&left_back_value, read_result.xel[LEFT_BACK].data, read_result.xel[LEFT_BACK].length);
    memcpy(&right_back_value, read_result.xel[RIGHT_BACK].data, read_result.xel[RIGHT_BACK].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t &left_front_value, int32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_front_value, read_result.xel[LEFT_FRONT].data, read_result.xel[LEFT_FRONT].length);
    memcpy(&right_front_value, read_result.xel[RIGHT_FRONT].data, read_result.xel[RIGHT_FRONT].length);
    memcpy(&left_back_value, read_result.xel[LEFT_BACK].data, read_result.xel[LEFT_BACK].length);
    memcpy(&right_back_value, read_result.xel[RIGHT_BACK].data, read_result.xel[RIGHT_BACK].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t &left_front_value, int16_t &right_front_value, int16_t &left_back_value, int16_t &right_back_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_front_value, read_result.xel[LEFT_FRONT].data, read_result.xel[LEFT_FRONT].length);
    memcpy(&right_front_value, read_result.xel[RIGHT_FRONT].data, read_result.xel[RIGHT_FRONT].length);
    memcpy(&left_back_value, read_result.xel[LEFT_BACK].data, read_result.xel[LEFT_BACK].length);
    memcpy(&right_back_value, read_result.xel[RIGHT_BACK].data, read_result.xel[RIGHT_BACK].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t &left_front_value, uint32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_front_value, read_result.xel[LEFT_FRONT].data, read_result.xel[LEFT_FRONT].length);
    memcpy(&right_front_value, read_result.xel[RIGHT_FRONT].data, read_result.xel[RIGHT_FRONT].length);
    memcpy(&left_back_value, read_result.xel[LEFT_BACK].data, read_result.xel[LEFT_BACK].length);
    memcpy(&right_back_value, read_result.xel[RIGHT_BACK].data, read_result.xel[RIGHT_BACK].length);
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::write_velocity(int32_t left_front_value, int32_t right_front_value, int32_t left_back_value, int32_t right_back_value)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT_FRONT].data, &left_front_value, sync_write_param.length);
  memcpy(sync_write_param.xel[LEFT_BACK].data, &left_back_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_FRONT].data, &right_front_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_BACK].data, &right_back_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t left_front_value, uint32_t right_front_value, int32_t left_back_value, int32_t right_back_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT_FRONT].data, &left_front_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_FRONT].data, &right_front_value, sync_write_param.length);
  memcpy(sync_write_param.xel[LEFT_BACK].data, &left_back_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_BACK].data, &right_back_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::control_motors(const float wheel_separation, float linear_x_value, float linear_y_value, float angular_value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];
  float lin_x_vel = linear_x_value;
  float lin_y_vel = linear_y_value;
  float ang_vel = angular_value;

  const float r = 0.05;
  const float wheel_separation_side = 0.08;

  // ORIGINAL
  /*wheel_velocity[LEFT_FRONT]   = lin_x_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT_FRONT]  = lin_x_vel + (ang_vel * wheel_separation / 2);
  wheel_velocity[LEFT_BACK]   = lin_x_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT_BACK]  = lin_x_vel + (ang_vel * wheel_separation / 2);*/

  wheel_velocity[LEFT_FRONT]   = (lin_x_vel - lin_y_vel - ((wheel_separation + wheel_separation_side) / 4) * ang_vel);
  wheel_velocity[RIGHT_FRONT]  = -(lin_x_vel + lin_y_vel + ((wheel_separation + wheel_separation_side) / 4) * ang_vel);
                              // ^ right front wheel moves opposite to others, temporarily fixed it here
  wheel_velocity[LEFT_BACK]   = (lin_x_vel + lin_y_vel - ((wheel_separation + wheel_separation_side) / 4) * ang_vel);
  wheel_velocity[RIGHT_BACK]  = (lin_x_vel - lin_y_vel + ((wheel_separation + wheel_separation_side) / 4) * ang_vel);

  wheel_velocity[LEFT_FRONT]  = constrain(wheel_velocity[LEFT_FRONT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT_FRONT] = constrain(wheel_velocity[RIGHT_FRONT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[LEFT_BACK]  = constrain(wheel_velocity[LEFT_BACK]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT_BACK] = constrain(wheel_velocity[RIGHT_BACK] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = write_velocity((int32_t)wheel_velocity[LEFT_FRONT], (int32_t)wheel_velocity[RIGHT_FRONT], (int32_t)wheel_velocity[LEFT_BACK], (int32_t)wheel_velocity[RIGHT_BACK]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
