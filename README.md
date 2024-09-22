# Mecanumbot

## Description

\[DESCRIPTION HERE]

## Setup

You can setup the Mecanumbot by following [the custom instructions](/setup.md).

## Usage

### 1. SSH to SBC
```bash
$ ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

### 2. Bring up basic packages
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ ros2 launch turtlebot3_bringup robot.launch.py
```

### 3. Control your Mecanumbot
Control the Mecanumbot by publishing messages on the `cmd_vel` topic. You can use [our custom made teleoperation node](https://github.com/Fortuz/mecanumbot/tree/main/mecanum_teleop).