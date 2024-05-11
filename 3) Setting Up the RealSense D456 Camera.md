# Install Intel RealSense ROS2 wrapper

  
#### Option 1: Install debian package from ROS servers (Foxy EOL distro is not supported by this option):
  - [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-realsense2-*```
  - For example, for Humble distro: ```sudo apt install ros-humble-realsense2-*```
  
#### Option 2: Install from source
  
  - Create a ROS2 workspace
      ```bash
      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws/src/
      ```
  
  - Clone the latest ROS2 Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
      ```bashrc
      git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
      cd ~/ros2_ws
      ```
  
  - Install dependencies
   ```bash
   sudo apt-get install python3-rosdep -y
   sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
   rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
   ```

  - Build
   ```bash
   colcon build
   ```

  -  Source environment
   ```bash
   ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: iron, humble, foxy
   source /opt/ros/$ROS_DISTRO/setup.bash
   cd ~/ros2_ws
   . install/local_setup.bash
   ```
  
  </details>

<hr>

