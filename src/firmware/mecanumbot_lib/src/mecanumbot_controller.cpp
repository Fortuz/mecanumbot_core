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

#include "../include/mecanumbot_controller.h"

MecanumbotController::MecanumbotController()
{
  const_cmd_vel_ = CONST_VEL;
}

MecanumbotController::~MecanumbotController()
{}

bool MecanumbotController::init(float max_lin_x_vel, float max_lin_y_vel, float max_ang_vel, uint8_t scale_lin_x_vel, uint8_t scale_lin_y_vel, uint8_t scale_ang_vel)
{
  // 57600bps baudrate for RC100 control
  rc100_.begin(1);  

  max_lin_x_vel_ = max_lin_x_vel;
  min_lin_x_vel_ = (-1)*max_lin_x_vel;
  max_lin_y_vel_ = max_lin_y_vel;
  min_lin_y_vel_ = (-1)*max_lin_y_vel;
  max_ang_vel_ = max_ang_vel;
  min_ang_vel_ = (-1)*max_ang_vel;
  scale_lin_x_vel_ = scale_lin_x_vel;
  scale_lin_y_vel_ = scale_lin_y_vel;
  scale_ang_vel_ = scale_ang_vel;

  return true;
}

void MecanumbotController::getRCdata(float *get_cmd_vel)
{
  uint16_t received_data = 0;

  static float lin_x = 0.0, lin_y = 0.0, ang_z = 0.0;
  
  if (rc100_.available())
  {
    received_data = rc100_.readData();

    if (received_data & RC100_BTN_U)
    {
      lin_x += VELOCITY_LINEAR_X * scale_lin_x_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      lin_x -= VELOCITY_LINEAR_X * scale_lin_x_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      lin_y += VELOCITY_LINEAR_Y * scale_lin_y_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      lin_y -= VELOCITY_LINEAR_Y * scale_lin_y_vel_;
    }
    else if (received_data & RC100_BTN_2)
    {
      ang_z += VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_4)
    {
      ang_z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      lin_x = const_cmd_vel_;
      lin_y = 0.0;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      lin_x = 0.0;
      lin_y = 0.0;
      ang_z = 0.0;
    }
    else
    {
      lin_x = lin_x;
      lin_y = lin_y;
      ang_z = ang_z;
    }

    lin_x = constrain(lin_x, min_lin_x_vel_, max_lin_x_vel_);
    lin_y = constrain(lin_y, min_lin_y_vel_, max_lin_y_vel_);
    ang_z = constrain(ang_z, min_ang_vel_, max_ang_vel_);

    get_cmd_vel[0] = lin_x;
    get_cmd_vel[1] = lin_y;
    get_cmd_vel[2] = ang_z;
  }
}
