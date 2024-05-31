#ifndef DRIVER_H_
#define DRIVER_H_

#include <DynamixelSDK.h>


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

#define DXL_NECK_YAW_ID                 13       // ID of yaw motor in the neck
#define DXL_NECK_PITCH_ID               14       // ID of pitch motor in the neck 

#define DXL_LEFT_GRABBER_ID                8       // ID of left rear motor
#define DXL_RIGHT_GRABBER_ID               18       // ID of right rear motor

#define BAUDRATE                        115200 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

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

#endif // DRIVER_H_
