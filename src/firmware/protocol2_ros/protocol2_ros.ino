/*
 * Protocol2 ROS test = MX motor test
 * 
 * callback based
 * 
 * Baudrate 115200 on every motor
 * AX1 - AX2 - MX1 - MX2 ------ OpenCR (serial connection) - Tested = Working!
 * 
 * roslaunch mecanum mecanum_port.launch
 * rostopic pub -1 /cmd_neck geometry_msgs/Vector3 -- '2000.0' '2000.0' '0.0'
 * 
 */

#include "config.h"

void setup() {
  DEBUG_SERIAL.begin(57600);

  nh.initNode();
  nh.getHardware()->setBaud(115200);

  // Subscribers
  nh.subscribe(neck_cmd);
  
  // Setting for Dynamixel motors
  neck_driver.init();

  pinMode(13, OUTPUT);
  
}

void loop() {
  nh.spinOnce();
}

/*******************************************************************************
* Callback function for cmd_neck
*******************************************************************************/
void cmdNeckCallback(const geometry_msgs::Vector3& cmd_neck){
  bool dxl_comm_result = false;
  
  dxl_comm_result = neck_driver.controlMotor((int64_t)cmd_neck.x, (int64_t)cmd_neck.y);
  if (dxl_comm_result == false)
    return;
}
