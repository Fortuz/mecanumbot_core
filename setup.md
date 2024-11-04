# Setup Mecanumbot

The majority of the Mecanumbot setup is identical to [the original Turtlebot3 instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), however there are a few custom modifications. Follow these instructions to setup the Mecanumbot, and please refer to the original instructions if you run into any issues.

**IMPORTANT:** if you have any issues and check out the original instructions on the Robotis webpage, choose `Foxy` at the top before reading any instructions.

## 1. PC Setup

### 1.1 Download and Install Ubuntu on PC

Download the proper `Ubuntu 20.04 LTS Desktop` image for your PC from [Ubuntu 20.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/20.04/).

Follow [the official instructions](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) to install Ubuntu on PC.

### 1.2 Install ROS 2 on Remote PC

Open the terminal with `Ctrl+Alt+T` and enter below commands one at a time.<br>
In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh).

```bash
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
$ sudo chmod 755 ./install_ros2_foxy.sh
$ bash ./install_ros2_foxy.sh
```
If the above installation fails, please refer to [the official ROS2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

### 1.3 Environment Configuration

Set the ROS environment for PC.
```bash
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc
```

If you have installed TurtleBot3 using Debian packages with apt install command, you can ignore the warning below.
```bash
bash: /home/{$YOUR_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory
```

## 2. SBC Setup

Follow [the official instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup).<br>
**IMPORTANT:** choose `Foxy` at the top before reading the instructions.

## 3. OpenCR Setup

### 3.1 Configure USB port for OpenCR
If you are using Linux, please configure the USB port for OpenCR. For other OS(OSX or Windows), you can skip this step.
```bash
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt install libncurses5-dev:i386
```

### 3.2 Download Arduino IDE
[Download](https://www.arduino.cc/en/software) and install the latest version of Arduino IDE.

### 3.3 Copy mecanumbot library
Navigate to `src/firmware` directory and copy the `mecanumbot_lib` folder to<br>
 - Windows: `C:\Users\{username}\Documents\Arduino\libraries\`
 - Linux: `/home/{username}/Arduino`

### 3.4 Run Arduino IDE.

### 3.5 Navigate to preferences
Press `Ctrl + ,` to open the Preferences menu (or go to `File > Preferences`).

### 3.6 Add Board Manager URL
Enter below address in the `Additional Boards Manager URLs`.
```
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
```

### 3.7 Open the `turtlebot3_mecanum/turtlebot3_mecanum.ino` file in Arduino IDE.
(If Arduino IDE does not find your library automatically, go to `Sketch > Include Library... > mecanumbot_lib`.) 

### 3.8 Setup board
Connect OpenCR to the PC and Select `OpenCR > OpenCR Board` from `Tools > Board` menu.

### 3.9 Setup port
Select the OpenCR connected USB port from `Tools > Port` menu.

### 3.10 Upload firmware
Upload the turtlebot3_mecanum firmware sketch with `Ctrl + U` or the upload icon.

## 4. Hardware Assembly

Please refer to [the official instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly) for the hardware assembly (`Assembly manual for TurtleBot3 Waffle Pi`).

## 5. Update code on SBC

### 5.1 Connect to SBC via SSH
```bash
$ ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```
Password: turtlebot

### 5.2 Navigate to source files
```bash
$ cd ~/turtlebot3_ws/src/turtlebot3/turtlebot3_node/src
```

### 5.3 Modify velocity callback function
In file `turtlebot3.cpp`, **line 325** (`cmd_vel_callback` function):

From this:
```cpp
data.dword[1] = 0;
```
Modify to this:
```cpp
data.dword[1] = static_cast<int32_t>(msg->linear.y * 100);
```

### 5.4 Add grabber callback function
In file `turtlebot3.cpp`, after **line 345** (at the end of the file), add:
```cpp
void TurtleBot3::grabber_goal_pos_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  grabber_goal_pos_sub_ = this->create_subscription<grabber_msg_interface::msg::GrabberPosition>(
    "grabber_goal_position",
    qos,
    [this](const grabber_msg_interface::msg::GrabberPosition::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int32_t dword[2];
        uint8_t byte[4 * 2];
      } data;

      data.dword[0] = static_cast<int32_t>(msg->l);
      data.dword[1] = static_cast<int32_t>(msg->r);

      uint16_t start_addr = extern_control_table.grabber_goal_position_l.addr;
      uint16_t addr_length =
      (extern_control_table.grabber_goal_position_r.addr -
      extern_control_table.grabber_goal_position_l.addr) +
      extern_control_table.grabber_goal_position_r.length;

      uint8_t * p_data = &data.byte[0];

      dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "grabber_goal_pos_l: %f grabber_goal_pos_r: %f msg : %s", msg->l, msg->r, sdk_msg.c_str());
    }
  );
}
```

### 5.5 Add callback to run method
In file `turtlebot3.cpp`, after **line 207** (in `run` function), add:
```cpp
grabber_goal_pos_callback();
```

### 5.6 Update memory
In file `turtlebot3.cpp`, at **line 68 to 72** (in `init_dynamixel_sdk_wrapper` function), modify this:
```cpp
dxl_sdk_wrapper_->init_read_memory(
  extern_control_table.millis.addr,
  (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
  extern_control_table.profile_acceleration_right.length
);
```
To this:
```cpp
dxl_sdk_wrapper_->init_read_memory(
  extern_control_table.millis.addr,
  (extern_control_table.grabber_goal_position_r.addr - extern_control_table.millis.addr) +
  extern_control_table.grabber_goal_position_r.length
);
```

### 5.7 Navigate to header files
```bash
$ cd ~/turtlebot3_ws/src/turtlebot3/turtlebot3_node/include/turtlebot3_node/
```

### 5.8 Add custom addresses at the end of control table
In file `control_table.hpp`, after **line 107** (inside the `ControlTable` struct) add the following lines:
```cpp
ControlItem grabber_goal_position_l = {348, RAM, 4, READ_WRITE};
ControlItem grabber_goal_position_r = {352, RAM, 4, READ_WRITE};
```

### 5.9 Update header file
In file `turtlebot3.hpp`, at the beginning add the following include:
```cpp
#include "grabber_msg_interface/msg/grabber_position.hpp"
```
later, at **line 96** add:
```cpp
void grabber_goal_pos_callback();
```
and at **line 114**, add:
```cpp
rclcpp::Subscription<grabber_msg_interface::msg::GrabberPosition>::SharedPtr grabber_goal_pos_sub_;
```

### 5.10 Add custom grabber msg
Copy `https://github.com/Fortuz/mecanumbot/tree/mecanum_teleop/grabber_msg_interface` folder to `~/turtlebot3_ws/src/`

### 5.11 Update CMakeLists.txt
Navigate to folder:
```bash
cd ~/turtlebot3_ws/src/turtlebot3/turtlebot3_node
```
In `CMakeListstxt`, **line 31** (at the end of other find_packages) add:
```
find_package(grabber_msg_interface REQUIRED)   
```
and **line 69** (at the end of dependencies), add:
```
"grabber_msg_interface"
```

### 5.12 Update package.xml
In `package.xml` after **line 29** add:
```xml
<depend>grabber_msg_interface</depend>
```

### 5.13 Build package
Navigate back to root of workspace
```bash
$ cd ~/turtlebot3_ws
```
Build package
```bash
$ colcon build
```

## 6. Try it out
You can now use the Mecanumbot by following the [bringup instructions](README.md/#usage).