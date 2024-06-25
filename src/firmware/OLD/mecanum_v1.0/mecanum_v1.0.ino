/*
 * Mecanum robot firmware for OpenCR
 */

#include "mecanum_config.h"

// 1. Turtlebot3 waffle minta
// mit publishol, hova stb
// topic: legyen odom
// 2 motor helyett 4 + matek

void setup()
{
  DEBUG_SERIAL.begin(57600);

  nh.initNode();
  nh.getHardware()->setBaud(115200); //115200
  
  // Subscribers
  nh.subscribe(sub_cmd);
  nh.subscribe(led_cmd);
  nh.subscribe(grabber_cmd);
  nh.subscribe(neck_cmd);
  
  // Publishers
  nh.advertise(voltage_pub);
  nh.advertise(led_pub);
  nh.advertise(grabber_pub);
  nh.advertise(neck_pub);
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(battery_state_pub);
  
  // Setting for Dynamixel motors
  motor_driver.init();
  grabber_driver.init();
  neck_driver.init();
  
  // Setting for IMU
  sensors.init();
  
  pinMode(13, OUTPUT);
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  controlLed();
  
  setup_end = true;
}

void loop()
{
  t = millis();

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)){
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) {
      goal_linear_x_velocity  = 0.0;
      goal_linear_y_velocity  = 0.0;
      goal_angular_velocity   = 0.0;
    } 
    controlMecanum();
    tTime[0] = t;
  }

  
  if ((t-tTime[1]) >= (1000 / CONTROL_NECK_FREQUENCY)){ 
    controlNeck();
    tTime[1] = t;
  }
  
  
  if ((t-tTime[2]) >= (1000 / VOLTAGE_PUBLISH_FREQUENCY )){
    voltage_msg.data = getPowerInVoltage();
    voltage_pub.publish(&voltage_msg);
    tTime[2] = millis();
  }

  if ((t-tTime[3]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishBatteryStateMsg();
    publishLedMsg();
    publishGrabberMsg();
    publishNeckMsg();
    tTime[3] = t;
  }
  
  if ((t-tTime[4]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[4] = t;
  }

  
  if ((t-tTime[5]) >= (1000 / CONTROL_GRABBER_FREQUENCY)){ 
    controlGrabber();
    tTime[5] = t;
  }
  
  
  // Update the IMU unit
  sensors.updateIMU();

  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);
  
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
  
}

/*******************************************************************************
* LED Callback, Publisher
*******************************************************************************/
void cmdLedCallback(const geometry_msgs::Vector3& cmd_led){
  led_r = cmd_led.x;
  led_g = cmd_led.y;
  led_b = cmd_led.z;
  // Timer implementation
  controlLed();
}

void controlLed(){
  analogWrite(REDPIN,   led_r);
  analogWrite(BLUEPIN,  led_b);
  analogWrite(GREENPIN, led_g);
}

void publishLedMsg(void)
{
  led_msg.x = led_r;
  led_msg.y = led_g;
  led_msg.z = led_b;
  
  led_pub.publish(&led_msg);
}
/*******************************************************************************
* Neck Callback, Function, Publisher
*******************************************************************************/
void cmdNeckCallback(const geometry_msgs::Vector3& cmd_neck){
  goal_yaw   = cmd_neck.x;
  goal_pitch = cmd_neck.y;
}

void controlNeck(){
  bool dxl_comm_result = false;

  if (goal_yaw > YAW_MAX)       goal_yaw = YAW_MAX;
  else if (goal_yaw < YAW_MIN)  goal_yaw = YAW_MIN;

  if (goal_pitch > PITCH_MAX)      goal_pitch = PITCH_MAX;
  else if (goal_pitch < PITCH_MIN) goal_pitch = PITCH_MIN;
  
  dxl_comm_result = neck_driver.controlMotor((int64_t)goal_yaw, (int64_t)goal_pitch);
  if (dxl_comm_result == false)
    return;
}


void publishNeckMsg(void)
{
  neck_msg.x = goal_yaw;
  neck_msg.y = goal_pitch;
  
  neck_pub.publish(&neck_msg);
}
/*******************************************************************************
* Grabber Callback, Function, Publisher
*******************************************************************************/
void cmdGrabberCallback(const geometry_msgs::Vector3& cmd_grabber){
  goal_grabber_left   = cmd_grabber.x;
  goal_grabber_right  = cmd_grabber.y;
}

void controlGrabber(){
  bool dxl_comm_result = false;

  if (goal_grabber_left > GRABBER_MAX)       goal_grabber_left = GRABBER_MAX;
  else if (goal_grabber_left < GRABBER_MIN)  goal_grabber_left = GRABBER_MIN;

  if (goal_grabber_right > GRABBER_MAX)      goal_grabber_right = GRABBER_MAX;
  else if (goal_grabber_right < GRABBER_MIN) goal_grabber_right = GRABBER_MIN;

  dxl_comm_result = grabber_driver.controlMotor((int64_t)goal_grabber_left, (int64_t)goal_grabber_right);
  if (dxl_comm_result == false)
    return;
}

void publishGrabberMsg(void)
{
  grabber_msg.x = goal_grabber_left;
  grabber_msg.y = goal_grabber_right;
  
  grabber_pub.publish(&grabber_msg);
}
/*******************************************************************************
* Mecanum drive Callback, Function
*******************************************************************************/
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  goal_linear_x_velocity  = cmd_vel.linear.x;
  goal_linear_y_velocity  = cmd_vel.linear.y;
  goal_angular_velocity   = cmd_vel.angular.z;
  tTime[6] = millis();
}

void controlMecanum()
{
  bool dxl_comm_result = false;

  int64_t wheel_value[MECANUMWHEEL_NUM] = {0, 0, 0, 0};
  double wheel_angular_velocity[MECANUMWHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[1] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[2] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[3] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);

  for (int id = 0; id < MECANUMWHEEL_NUM; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * 9.54 / RPM_CONSTANT_VALUE;

    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

  dxl_comm_result = motor_driver.controlMotor(-(int64_t)wheel_value[0], -(int64_t)wheel_value[1], (int64_t)wheel_value[2], (int64_t)wheel_value[3]);
  if (dxl_comm_result == false)
    return;
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}
