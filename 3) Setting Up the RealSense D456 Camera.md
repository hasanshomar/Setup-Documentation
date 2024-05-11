# Install Intel RealSense ROS2 wrapper

  

#### Install from source
  
  - Create a ROS2 workspace
```bash
mkdir -p ~/ros2_ws/src
```
```bash
cd ~/ros2_ws/src/
```
  
  - Clone the latest ROS2 Intel RealSense  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
```bashrc
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
```
```bashrc
cd ~/ros2_ws
```
  
  - Install dependencies
```bash
sudo apt-get install python3-rosdep -y
```
```bash
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
```
```bash
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
```
```bash
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```

  - Build
```bash
colcon build
```

  -  Source environment
```bash
ROS_DISTRO=foxy
```
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```
```bash
cd ~/ros2_ws
```
```bash
. install/local_setup.bash
```
  
  </details>

<hr>

