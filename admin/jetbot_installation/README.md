# Installing software on Jetbot

**ONLY FOR THE TEACHER**

This section will go through all necessary steps to install software on Jetson Nano/Jetbot for MAS514. 
There are two major steps involved:

1. Installing operating system
   - Format SD card
   - Set communication settings on Jetbot
2. Installing Jetbot software
   - Python
   - OpenCV
   - ROS

These steps are normally completed before the course starts. 
Note that your group number (`<group-number>`, Jetbot number) is needed for the setup.

# 1. Installing operating system

This step requires a monitor with HDMI or DisplayPort, a keyboard and a mouse. 
A separate computer is used to download image and flash SD card.

## Format SD card

The steps are as follows:

- Follow the instructions [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) for writing the ISO file to the SD card.
- Put SD card back into Jetbot.
- Connect monitor, mouse and keyboard to Jetbot.
- Connect ethernet cable for updates.
- Connect power to Jetbot.
- Once booted, the installation process can proceed.

During the installation, these options should be made:

- User name: `jetbot`
- Password: `jetbot`
- Computer name: `jetbot-desktop<group-number>`
  - For example, for group 3 (box with 3 on): jetbot-desktop3

## Internet access

- If possible, attach an ethernet cable to the Jetbot for installing updates and software. 
- Alternatively, use Windows hotspot ?? 

## Updates

After installation, you can update the system with 

- `sudo apt update && sudo apt -y upgrade && sudo apt -y install network-manager`

## Fixed IP

The ethernet port is set to static IP for communication with any laptop. 

- Open a bash terminal: `sudo nm-connection-editor`.
- Edit the wired connection. 
- Click on `IPv4 Settings` tab.
- Set method to `Manual` and add a new address:
  - Address: `192.168.0.<group-number>`
    - For example 192.168.0.3 for group 3 (box with 3 on)
  - Netmask: `255.255.255.0`
  - Gateway: BLANK
- Click on "Routes" button in lower right corner. 
- Activate the "Use this connection only for resources on its network". 
  - This will make sure that Ubuntu only uses LAN for local connections, and will not try to use it for internet access. 
- Click OK, Save etc.

## BashRC edit

Paste the content of `jetbot.bash` into `~/.bashrc` at the end. 

## Make SSH keys

SSH keys come in pairs: a private and a public key.
The public key resides on the server (jetbot), and the private key with the user (client)
SSH keys are helpful for opening a remote SSH session without being prompted for a password.
It is also possible to revoke access with password, and require the private key for remote access.
This is very helpful to maximize safety for remote systems. 

- Navigate to `~\.ssh` in a bash terminal on the jetbot
  - Make new keys with `ssh-keygen -t rsa -C "jetbot<group-number>" -f id_rsa_jetbot<group-number>`
  - Give no password (empty passphrase)
  - Make a new file called `authorized_keys` and paste all the content (including the newline) from `id_rsa_jetbot<group-number>` into this file.
    - This will tell the system that the owner of the private key may access through SSH without asking for a password.

# 2. Installing Jetbot software

## Installing necessary software for Python

Installing system tools over SSH:

`sudo apt install -y python-pip libi2c-dev i2c-tools python-pil python-tk dos2unix ipython`

Python packages (Copy **one** line at a time):

``` {.console}
pip install numpy
pip install matplotlib
pip install Jetson.GPIO
pip install Adafruit_PCA9685
```

## OpenCV 4.1.1

Check first if OpenCV 4.1.1 is installed already

- Run `python -c "import cv2; import cv2.aruco; print(cv2.__version__)"`
  - If successful, it should not crash, and 4.1.1 should be printed.
  - If it crashes or another version (like 3.3.1) is printed, proceed to installation below

- Add Swap Space.
  - [Based on](https://linuxize.com/post/how-to-add-swap-space-on-ubuntu-18-04/?fbclid=IwAR2Sm3NynkVUhTxOacU2ssDvGz21bl01mkJL6RMCTNP5KTmQFw9BMZFZUCo)
  - Before continuing with this tutorial, check if your Ubuntu installation already has swap enabled by typing: `sudo swapon --show`
  - Start by creating a file which will be used for swap: `sudo fallocate -l 4G /swapfile`
  - Only the root user should be able to write and read the swap file. Set the correct permissions by typing: `sudo chmod 600 /swapfile`
  - Use the mkswap utility to set up a Linux swap area on the file: `sudo mkswap /swapfile`
  - Activate the swap file using the following command: `sudo swapon /swapfile`
  - Verify that the swap is active by using either the swapon or the free command , as shown below: `sudo swapon --show`

Installation instructions that requires internet on jetbot:

- Move `jetbot-files/installOpenCV4.1.1.sh` to `/home/jetbot` by using VSCode (Move the file to the file explorer tree)
- Make directory on jetbot for installation
  - `mkdir -p ~/build_opencv`
- Then run installation with
  - `dos2unix ~/installOpenCV4.1.1.sh && chmod +x ~/installOpenCV4.1.1.sh && sudo ~/installOpenCV4.1.1.sh "/home/jetbot/build_opencv"`
- This will take a long time (Maybe 4 hours)
- After finishing, please check if installed correctly by doing
  - `python -c "import cv2; import cv2.aruco; print(cv2.__version__)"`
    - Should say `4.1.1`.

## GPIO

GPIO are general purpose IO pins, which are the 40 pins on the Jetbot.
We need access to them for communicating with motor driver.
By default, only root user can access them, but it should be opened by the normal jetbot user.
Follow these instructions (as based on [JetbotHacks](https://www.jetsonhacks.com/2019/06/07/jetson-nano-gpio/)).

- Open a new terminal.
- Add `jetbot` user to list for accessing GPIO devices:
  - `sudo groupadd -f -r gpio && sudo usermod -a -G gpio jetbot`
- Copy custom rules to another folder:
  - `sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/`
- Reload rules:
  - `sudo udevadm control --reload-rules && sudo udevadm trigger`
- Add jetbot user to i2c group for permission:
  - `sudo adduser jetbot i2c`
- Reboot for the changes to be in effect:
  - `sudo reboot`

## Check motor communication

The motor controller Adafruit_PCA9685 is connected to I2C bus 1.
To check if the communication is working, try to detect all I2C devices on bus 1:

`sudo i2cdetect -y -r 1`

Possible output if motor is connected successfully. The 40 register means the motor driver.

![](./figs/motor/i2cbus_motor.png)

If not succeeded, try to re-attach wires to motor driver.

## Robot Operating System

Install ROS Melodic for Ubuntu 18.04 using the commands below. 
These commands are taken from [ROS website](http://wiki.ros.org/melodic/Installation/Debian).

- `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
- `sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
- `sudo apt update && sudo apt -y install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-web-video-server`
- `sudo rosdep init && rosdep update && source ~/.bashrc`
- `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make`

## Robot Upstart
Install robot_upstart to enable for `roscore_service` to work
- `sudo apt-get install ros-melodic-robot-upstart`

## Install Rosbridge for WebViz to work
In order for WebViz to work, the Rosbridge has to be installed on the Jetbot. The installation is carried our by simply executing the following command.
- `sudo apt-get install ros-melodic-rosbridge-suite`
