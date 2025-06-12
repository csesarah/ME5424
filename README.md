# ME5424
Ubuntu 22.04, ROS 2 Humble, Gazebo Ignition Fortress

## Dependencies
Respositories

        sudo apt install git wget curl gnupg 
        export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg
        curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
        sudo apt install /tmp/ros2-apt-source.deb

Install

        sudo apt update && sudo apt upgrade
        sudo apt install build-essential cmake software-properties-common
        sudo apt install python3 python3-colcon-common-extensions 
        sudo apt install libeigen3-dev libopencv-dev libyaml-cpp-dev libcurl4-openssl-dev protobuf-compiler libprotobuf-dev
        sudo apt install openjdk-18-jre openjdk-18-jdk
        sudo apt install ros-humble-desktop ros-humble-xacro ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
        sudo apt upgrade

        cd src
        
        sudo rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src

### PX4
        git clone https://github.com/PX4/PX4-Autopilot.git
        git clone https://github.com/PX4/px4_ros_com.git
        git clone https://github.com/PX4/px4_msgs.git

Build

        cd PX4-Autopilot
        bash ./Tools/setup/ubuntu.sh
        make clean
        make px4_sitl


### Micro-XRCE
        git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        cd Micro-XRCE-DDS-Agent

Edit ```CMakeLists.txt```, line 98-99: 

        set(_fastdds_version 2.13)
        set(_fastdds_tag v2.13.6)

Build

        mkdir build
        cd build
        cmake ..
        make
        sudo make install
        sudo ldconfig /usr/local/lib/

---

## Usage
        cd ..
        rosdep update

        export AMENT_PREFIX_PATH=$(pwd)/install:$AMENT_PREFIX_PATH
        source /opt/ros/humble/setup.bash

        colcon build --symlink-install --merge-install
        ros2 pkg list | grep bswarm

In terminal 1:

        source install/setup.bash
        cd src
        ros2 launch bswarm swarm.launch.py

In terminal 2:

        source install/setup.bash
        cd src
        ros2 run bswarm team_controller.py

