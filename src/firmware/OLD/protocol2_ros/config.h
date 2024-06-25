#ifndef CONFIG_H_
#define CONFIG_H_

#define NOETIC_SUPPORT

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <math.h>

#include "driver.h"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define FIRMWARE_VER                     "1.1"

static uint32_t tTime[10];                // Software timer
uint32_t t = 0;

double yaw = 0.0;
double pitch = 0.0;

ros::NodeHandle nh;

NeckMotorDriver neck_driver;

void cmdNeckCallback(const geometry_msgs::Vector3& cmd_neck);

ros::Subscriber<geometry_msgs::Vector3> neck_cmd("cmd_neck", &cmdNeckCallback);

#endif // CONFIG_H_
