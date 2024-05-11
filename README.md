Setup-Documentation
======================

Setup Documentation for ROS 2 and RealSense Camera Integration and Configuration
-----------------------------------------------------------------------------------

This table provides a comprehensive overview of the hardware and software requirements necessary for setting up and operating the ROS 2 and Intel RealSense integration project. It details the specific computer specifications, supported operating system, versions of essential software including the ROS 2 distribution and RealSense SDK, and additional dependencies required for successful implementation. Additionally, it lists the model and firmware version of the RealSense camera used in the project. This information is crucial for replicating the setup in similar environments and ensures compatibility and optimal performance of all components.

| **Requirement Type** | **Description**                      | **Details/Version**   |
|----------------------|--------------------------------------|-----------------------|
| Hardware             | Computer Specifications              | (Processor, RAM, Storage) |
| Operating System     | Supported OS                         | Ubuntu 20.04 LTS      |
| Software             | ROS 2 Distribution                   | Foxy           |
|                      | RealSense SDK                        | 2.55.1.0              |
| Dependencies         | Development Tools and Libraries      | Python 3.8, CMake, etc. |
| Camera               | Model                                | RealSense D456        |
|                      | Firmware Version                     | (Version if applicable) |

## 1. Set Locale
-------------
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```
locale  # check for UTF-8
```
```
sudo apt update && sudo apt install locales
```
```
sudo locale-gen en_US en_US.UTF-8
```
```
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```
```
export LANG=en_US.UTF-8
```
```
locale  # verify settings
```
## 2. Setup Sources
----------------
You will need to add the ROS 2 apt repository to your system.
First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
```
```
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
```
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
## 3. Install ROS 2 Packages
----------------
Update your apt repository caches after setting up the repositories.
```
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```
sudo apt upgrade
```
Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```
sudo apt install ros-foxy-desktop python3-argcomplete
```
ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
```
sudo apt install ros-foxy-ros-base python3-argcomplete
```
Development tools: Compilers and other tools to build ROS packages
```
sudo apt install ros-dev-tools
```
## 4. Environment Setup 
----------------
### Sourcing the setup script
Set up your environment by sourcing the following file.
```
source /opt/ros/foxy/setup.bash
```

