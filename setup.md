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
```bash
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
Password: *** (TODO will it be displayed here ?)

### 5.2 Navigate to code (TODO exact file ?)
```bash
$ cd ~/turtlebot3_ws/src/turtlebot3/turtlebot3_node/src
```

### 5.3 Modify velocity callback function (TODO exact file exact lines ?)
In file `?.py` line ?-?:
From this: (TODO)
```
???
```
Modify to this: (TODO)
```
???
```

### 5.4 Build package
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