# Setup Mecanumbot

## 1. PC Setup

### 1.1.1 Download and Install Ubuntu on PC

Download the proper `Ubuntu 20.04 LTS Desktop` image for your PC from [Ubuntu 20.04 LTS Desktop image (64-bit)](https://releases.ubuntu.com/20.04/).

Follow [the official instructions](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) to install Ubuntu on PC.

### 1.1.2 Install ROS 2 on Remote PC

Open the terminal with `Ctrl+Alt+T` and enter below commands one at a time.<br>
In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh).

```bash
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
$ sudo chmod 755 ./install_ros2_foxy.sh
$ bash ./install_ros2_foxy.sh
```
If the above installation fails, please refer to the official ROS2 Foxy installation guide.

### 1.1.3 Install Dependent ROS 2 Packages - TODO: this is not necessary for us I think?

Open the terminal with `Ctrl+Alt+T` from Remote PC.

Install Gazebo11
```bash
$ sudo apt-get install ros-foxy-gazebo-*
```
Install Cartographer
```bash
$ sudo apt install ros-foxy-cartographer
$ sudo apt install ros-foxy-cartographer-ros
```
Install Navigation2
```bash
$ sudo apt install ros-foxy-navigation2
$ sudo apt install ros-foxy-nav2-bringup
```

### 1.1.4 Install TurtleBot3 Packages - TODO: this is not necessary for us I think?

Install TurtleBot3 via Debian Packages.
```bash
$ source ~/.bashrc
$ sudo apt install ros-foxy-dynamixel-sdk
$ sudo apt install ros-foxy-turtlebot3-msgs
$ sudo apt install ros-foxy-turtlebot3
```

### 1.1.5 Environment Configuration

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
**IMPORTANT:** choose `Foxy` at the top.

## 3. OpenCR Setup

**TODO**

## 4. Hardware Assembly

**TODO**