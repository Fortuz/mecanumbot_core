#include "mecanum_config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop(); }}

void setup()
{
    // ???
    // Serial.begin(115200);

    // Init micro-ROS transport
    set_microros_transports();

    // Delay initialization
    delay(2000);

    // Set default allocator
    allocator = rcl_get_default_allocator();

    // Create init options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Initialize node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    
    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Setting for Dynamixel motors
    motor_driver.init();

    setup_end = true;
}

void loop() {
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

    // Spin executor
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // Add small delay for loop control
    delay(100);

    // ???
    // waitForSerialLink(nh.connected());
}

/*******************************************************************************
* Mecanum drive Callback, Function
*******************************************************************************/
void cmdVelCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    goal_linear_x_velocity  = cmd_vel->linear.x;
    goal_linear_y_velocity  = cmd_vel->linear.y;
    goal_angular_velocity   = cmd_vel->angular.z;
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
* Error loop
*******************************************************************************/
void errorLoop(){
  while(1){
    delay(100);
  }
}