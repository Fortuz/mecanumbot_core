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
  uint8_t left_front_wheel_id_;
  uint8_t right_front_wheel_id_;
  uint8_t left_back_wheel_id_;
  uint8_t right_back_wheel_id_;
  bool torque_;
};

#endif // MECANUMBOT_MOTOR_DRIVER_H_
