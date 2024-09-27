#include <mecanum_lib.h>

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Begin Mecanumbot core for support Mecanum.
  MecanumbotCore::begin("Mecanum");
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  // Run TurtleBot3 core for communicating with ROS2 node, sensing several sensors and controlling actuators.
  MecanumbotCore::run();
}