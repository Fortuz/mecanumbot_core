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

#ifndef MECANUM_MOTOR_DRIVER_H_
#define MECANUM_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

/*******************************************************************************
*  Common Constants
*********************************************************************************/
#define BAUDRATE                        115200  // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

/*********************************************************************************
 * Motor constants for Mecanum Drive and Neck
 *********************************************************************************/
// Control table address for XM430-W210-T (Dynamixel X-series)
// ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
#define PROTOCOL_VERSION2               2.0     // Dynamixel protocol version 2.0

#define ADDR_X_TORQUE_ENABLE            64   // RW
#define ADDR_X_LED                      65   // RW
#define ADDR_X_STATUS_RETURN_LEVEL      68   // RW
#define ADDR_X_REGISTERED_INSTRUCTION   69   // R
#define ADDR_X_HARDWARE_ERROR_STATUS    70   // R
#define ADDR_X_VELOCITY_I_GAIN          76   // RW
#define ADDR_X_VELOCITY_P_GAIN          78   // RW
#define ADDR_X_POSITION_D_GAIN          80   // RW
#define ADDR_X_POSITION_I_GAIN          82   // RW
#define ADDR_X_POSITION_P_GAIN          84   // RW
#define ADDR_X_FEEDFORWARD_2nd_GAIN     88   // RW
#define ADDR_X_FEEDFORWARD_1st_GAIN     90   // RW
#define ADDR_X_BUS_WATCHDOG             98   // RW
#define ADDR_X_GOAL_PWM                 100  // RW
#define ADDR_X_GOAL_CURRENT             102  // RW
#define ADDR_X_GOAL_VELOCITY            104  // RW   
#define ADDR_X_PROFILE_ACCELERATION     108  // RW
#define ADDR_X_PROFILE_VELOCITY         112  // RW
#define ADDR_X_GOAL_POSITION            116  // RW
#define ADDR_X_REALTIME_TICK            120  // R
#define ADDR_X_MOVING                   122  // R
#define ADDR_X_MOVING_STATUS            123  // R
#define ADDR_X_PRESENT_PWM              124  // R
#define ADDR_X_PRESENT_CURRENT          126  // R
#define ADDR_X_PRESENT_VELOCITY         128  // R
#define ADDR_X_PRESENT_POSITION         132  // R
#define ADDR_X_VELOCITY_TRAJECTORY      136  // R
#define ADDR_X_POSITION_TRAJECTORY      140  // R
#define ADDR_X_PRESENT_INPUT_VOLTAGE    144  // R
#define ADDR_X_PRESENT_TEMPERATURE      146  // R
#define ADDR_X_BACKUP_READY             147  // R

// Data Byte Length
#define LEN_X_TORQUE_ENABLE            1   
#define LEN_X_LED                      1   
#define LEN_X_STATUS_RETURN_LEVEL      1   
#define LEN_X_REGISTERED_INSTRUCTION   1   
#define LEN_X_HARDWARE_ERROR_STATUS    1   
#define LEN_X_VELOCITY_I_GAIN          2   
#define LEN_X_VELOCITY_P_GAIN          2   
#define LEN_X_POSITION_D_GAIN          2   
#define LEN_X_POSITION_I_GAIN          2   
#define LEN_X_POSITION_P_GAIN          2   
#define LEN_X_FEEDFORWARD_2nd_GAIN     2   
#define LEN_X_FEEDFORWARD_1st_GAIN     2   
#define LEN_X_BUS_WATCHDOG             1   
#define LEN_X_GOAL_PWM                 2  
#define LEN_X_GOAL_CURRENT             2  
#define LEN_X_GOAL_VELOCITY            4     
#define LEN_X_PROFILE_ACCELERATION     4  
#define LEN_X_PROFILE_VELOCITY         4  
#define LEN_X_GOAL_POSITION            4  
#define LEN_X_REALTIME_TICK            2  
#define LEN_X_MOVING                   1  
#define LEN_X_MOVING_STATUS            1  
#define LEN_X_PRESENT_PWM              2  
#define LEN_X_PRESENT_CURRENT          2  
#define LEN_X_PRESENT_VELOCITY         4  
#define LEN_X_PRESENT_POSITION         4  
#define LEN_X_VELOCITY_TRAJECTORY      4  
#define LEN_X_POSITION_TRAJECTORY      4  
#define LEN_X_PRESENT_INPUT_VOLTAGE    2  
#define LEN_X_PRESENT_TEMPERATURE      1  
#define LEN_X_BACKUP_READY             1  

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

#define ENCODER_MIN                    -2147483648     // raw
#define ENCODER_MAX                     2147483648     // raw

#define DXL_LEFT_REAR_ID                1       // ID of left rear motor
#define DXL_RIGHT_REAR_ID               2       // ID of right rear motor
#define DXL_LEFT_FRONT_ID               3       // ID of left front motor
#define DXL_RIGHT_FRONT_ID              4       // ID of right front motor

#define DXL_NECK_YAW_ID                 7       // ID of yaw motor in the neck
#define DXL_NECK_PITCH_ID               8       // ID of pitch motor in the neck 

// Limits for Neck
#define PITCH_MAX                       3072    // Look up
#define PITCH_MIN                       1024    // Look down
#define YAW_MAX                         3072    // Look left
#define YAW_MIN                         1024    // Look right
/********************************************************************************* 
*  Motor Constants for Grabber
**********************************************************************************/
// Control table address for Dynamixel AX-12A
// ref: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
#define PROTOCOL_VERSION1                1.0 // Dynamixel protocol version 2.0

#define ADDR_AX_TORQUE_ENABLE            24  // RW
#define ADDR_AX_LED                      25  // RW
#define ADDR_AX_CW_COMPLIANCE_MARGIN     26  // RW
#define ADDR_AX_CCW_COMPLIANCE_MARGIN    27  // RW
#define ADDR_AX_CW_COMPLIANCE_SLOPE      28  // RW
#define ADDR_AX_CCW_COMPLIANCE_SLOPE     26  // RW
#define ADDR_AX_GOAL_POSITION            30  // RW
#define ADDR_AX_MOVING_SPEED             32  // RW
#define ADDR_AX_TORQUE_LIMIT             34  // RW
#define ADDR_AX_PRESENT_POSITION         36  // R
#define ADDR_AX_PRESENT_SPEED            38  // R
#define ADDR_AX_PRESENT_LOAD             40  // R
#define ADDR_AX_PRESENT_VOLTAGE          42  // R
#define ADDR_AX_PRESENT_TEMPERATURE      43  // R
#define ADDR_AX_REGISTERED               44  // R
#define ADDR_AX_MOVING                   46  // R
#define ADDR_AX_LOCK                     47  // RW
#define ADDR_AX_PUNCH                    48  // RW

// Data Byte Length
#define LEN_AX_TORQUE_ENABLE             1
#define LEN_AX_LED                       1
#define LEN_AX_CW_COMPLIANCE_MARGIN      1
#define LEN_AX_CCW_COMPLIANCE_MARGIN     1
#define LEN_AX_CW_COMPLIANCE_SLOPE       1
#define LEN_AX_CCW_COMPLIANCE_SLOPE      1
#define LEN_AX_GOAL_POSITION             2
#define LEN_AX_MOVING_SPEED              2
#define LEN_AX_TORQUE_LIMIT              2
#define LEN_AX_PRESENT_POSITION          2
#define LEN_AX_PRESENT_SPEED             2
#define LEN_AX_PRESENT_LOAD              2
#define LEN_AX_PRESENT_VOLTAGE           1
#define LEN_AX_PRESENT_TEMPERATURE       1
#define LEN_AX_REGISTERED                1
#define LEN_AX_MOVING                    1
#define LEN_AX_LOCK                      1
#define LEN_AX_PUNCH                     2

#define DXL_LEFT_GRABBER_ID                5       // ID of left rear motor
#define DXL_RIGHT_GRABBER_ID               6       // ID of right rear motor

#define GRABBER_MIN                      400       // 512= straigt forward      
#define GRABBER_MAX                      624

/********************************************************************************
 * Class definition for Mecanum Drive Platform
 ********************************************************************************/
class MecanumMotorDriver
{
 public:
  MecanumMotorDriver();
  ~MecanumMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  bool controlMotor(int64_t left_rear_wheel_value, int64_t right_rear_wheel_value, int64_t left_front_wheel_value, int64_t right_front_wheel_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_rear_wheel_id_, right_rear_wheel_id_;
  uint8_t left_front_wheel_id_, right_front_wheel_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
};

/********************************************************************************
 * Class definition for Neck
 ********************************************************************************/
class NeckMotorDriver
{
 public:
  NeckMotorDriver();
  ~NeckMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool controlMotor(int64_t neck_yaw_value, int64_t neck_pitch_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t neck_yaw_id_, neck_pitch_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
};
/********************************************************************************
 * Class definition for Grabber Motor Control
 ********************************************************************************/
class GrabberMotorDriver
{
 public:
  GrabberMotorDriver();
  ~GrabberMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool controlMotor(int64_t left_grabber_value, int64_t right_grabber_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_grabber_id_, right_grabber_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
};

#endif // MECANUM_MOTOR_DRIVER_H_
