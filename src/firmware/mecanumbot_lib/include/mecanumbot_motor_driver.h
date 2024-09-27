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

#ifndef MECANUMBOT_MOTOR_DRIVER_H_
#define MECANUMBOT_MOTOR_DRIVER_H_

#include <Dynamixel2Arduino.h>

#define TORQUE_ENABLE ControlTableItem::TORQUE_ENABLE

enum MotorLocation{
  LEFT_FRONT = 0,
  RIGHT_FRONT,
  LEFT_BACK,
  RIGHT_BACK,
  MOTOR_NUM_MAX
};

enum VelocityType{
  LINEAR_X = 0,
  LINEAR_Y,
  ANGULAR,
  TYPE_NUM_MAX
};


class MecanumbotMotorDriver
{
 public:
  MecanumbotMotorDriver();
  ~MecanumbotMotorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();

  bool set_torque(bool onoff);
  bool get_torque();
  
  bool read_present_position(int32_t &left_front_value, int32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value);
  bool read_present_velocity(int32_t &left_front_value, int32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value);
  bool read_present_current(int16_t &left_front_value, int16_t &right_front_value, int16_t &left_back_value, int16_t &right_back_value);
  bool read_profile_acceleration(uint32_t &left_front_value, uint32_t &right_front_value, int32_t &left_back_value, int32_t &right_back_value);
  
  bool write_velocity(int32_t left_front_value, int32_t right_front_value, int32_t left_back_value, int32_t right_back_value);
  bool write_profile_acceleration(uint32_t left_front_value, uint32_t right_front_value, int32_t left_back_value, int32_t right_back_value);

  bool control_motors(const float wheel_separation, float linear_x_value, float linear_y_value, float angular_value);

  Dynamixel2Arduino& getDxl();
  
 private:
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
  const uint32_t DXL_PORT_BAUDRATE = 1000000; // Baud rate of Dynamixel
  const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

  ParamForSyncReadInst_t sync_read_param;
  ParamForSyncWriteInst_t sync_write_param;
  RecvInfoFromStatusInst_t read_result;
  Dynamixel2Arduino dxl;

  uint8_t left_front_wheel_id_;
  uint8_t right_front_wheel_id_;
  uint8_t left_back_wheel_id_;
  uint8_t right_back_wheel_id_;
  bool torque_;
};

#endif // MECANUMBOT_MOTOR_DRIVER_H_
