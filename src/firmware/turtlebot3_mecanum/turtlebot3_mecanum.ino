#include <TurtleBot3_ROS2_mecanum.h>

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Begin TurtleBot3 core for support Mecanum.
  TurtleBot3Core::begin("Mecanum");
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  // Run TurtleBot3 core for communicating with ROS2 node, sensing several sensors and controlling actuators.
  TurtleBot3Core::run();
}