#ifndef MECANUM_CONFIG_H_
#define MECANUM_CONFIG_H_

#define NOETIC_SUPPORT

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <math.h>

#include "mecanum_motor_driver.h"

// TurtleBot3 dependent imports
#include <TurtleBot3.h>


// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define FIRMWARE_VER                     "1.5"

/*******************************************************************************
* Update frequencies and Timing
*******************************************************************************/
#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_NECK_FREQUENCY                 30   //hz
#define CONTROL_GRABBER_FREQUENCY              50   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VOLTAGE_PUBLISH_FREQUENCY              50   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

/*******************************************************************************
* Robot parameters
*******************************************************************************/
#define NAME                            "Mecanum"

/* --------------------------- TODO ------------------------------------------*/
#define WHEEL_RADIUS                    0.03      // meter
#define WHEEL_SEPARATION_X              0.1005    // 170mm/2 + 31mm/2
#define WHEEL_SEPARATION_Y              0.085     // 200mm/2 - 30mm/2
#define DISTANCE_CENTER_TO_WHEEL        0.165     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define RPM_CONSTANT_VALUE              0.229

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_LINEAR_Y               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_LINEAR_Y         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define MECANUMWHEEL_NUM                4
#define LIMIT_X_MAX_VALUE               480
/*----------------------------------------------------------------------------*/
#define CONTROL_PERIOD                  8000

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                        0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define REDPIN 6
#define GREENPIN 5
#define BLUEPIN 3

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char imu_frame_id[30];
char mag_frame_id[30];

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
GrabberMotorDriver grabber_driver;
double goal_grabber_left   = 512.0;
double goal_grabber_right  = 512.0;

NeckMotorDriver neck_driver;
double goal_yaw    = 2048.0;
double goal_pitch  = 2048.0;

MecanumMotorDriver motor_driver;
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity   = 0.0;

static uint32_t tTime[10];                // Software timer
uint32_t t = 0;

Turtlebot3Sensor sensors;
Turtlebot3Diagnosis diagnosis;

bool setup_end        = false;
uint8_t battery_state = 0;

double led_r = 0.0;
double led_g = 5.0;
double led_b = 0.0;

/*******************************************************************************
* Callback function prototypes
*******************************************************************************/
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
void cmdLedCallback(const geometry_msgs::Vector3& cmd_led);
void cmdGrabberCallback(const geometry_msgs::Vector3& cmd_grabber);
void cmdNeckCallback(const geometry_msgs::Vector3& cmd_neck);
/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", &cmdVelCallback);
ros::Subscriber<geometry_msgs::Vector3> led_cmd("cmd_led", &cmdLedCallback);
ros::Subscriber<geometry_msgs::Vector3> grabber_cmd("cmd_grabber", &cmdGrabberCallback);
ros::Subscriber<geometry_msgs::Vector3> neck_cmd("cmd_neck", &cmdNeckCallback);
/*******************************************************************************
* Publisher
*******************************************************************************/
geometry_msgs::Vector3 led_msg;
ros::Publisher led_pub("rob_led", &led_msg);

geometry_msgs::Vector3 grabber_msg;
ros::Publisher grabber_pub("rob_grabber", &grabber_msg);

geometry_msgs::Vector3 neck_msg;
ros::Publisher neck_pub("rob_neck", &neck_msg);

std_msgs::Float32 voltage_msg;
ros::Publisher voltage_pub("rob_voltage", &voltage_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("rob_imu", &imu_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("rob_mag", &mag_msg);

// Battey state of Turtlebot3
#if defined NOETIC_SUPPORT
sensor_msgs::BatteryStateNoetic battery_state_msg;
#else
sensor_msgs::BatteryState battery_state_msg;
#endif
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

/*******************************************************************************
* Function prototypes
*******************************************************************************/
ros::Time rosNow(void);

void controlMecanum();

void publishImuMsg(void);
void publishMagMsg(void);
void publishBatteryStateMsg(void);

#endif // MECANUM_CONFIG_H_

// Ref : H.Taheri, B.Qiao, N.Ghaeminezhad, "Kinematic Model of a Four Mecanum Wheeled Mobile Robot",
//       International Journal of Computer Applications, 3 March 2015
