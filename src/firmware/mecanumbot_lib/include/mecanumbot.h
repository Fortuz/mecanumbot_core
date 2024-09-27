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

#ifndef MECANUMBOT_H_
#define MECANUMBOT_H_

#include <stdint.h>
#include "mecanumbot_motor_driver.h"
#include "mecanumbot_sensor.h"
#include "mecanumbot_controller.h"
#include "mecanumbot_diagnosis.h"
#include "open_manipulator_driver.h"
#include "mecanumbot_grabber.h"

#define DEBUG_ENABLE 1

#if DEBUG_ENABLE
  #define DEBUG_SERIAL_BEGIN(x) SerialBT2.begin(x)
  #define DEBUG_PRINT(x) SerialBT2.print(x)
  #define DEBUG_PRINTLN(x) SerialBT2.println(x)
#else
  #define DEBUG_SERIAL_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

const uint8_t FIRMWARE_VER = 5; //DYNAMIXEL2Arduino v0.6.1 or higher is required.
const uint32_t INTERVAL_MS_TO_CONTROL_MOTOR = 20;
const uint32_t INTERVAL_MS_TO_UPDATE_CONTROL_ITEM = 20;

namespace MecanumbotCore{
  void begin(const char* model_name);
  void run();
} //namespace MecanumbotCore



#endif // MECANUMBOT_H_

