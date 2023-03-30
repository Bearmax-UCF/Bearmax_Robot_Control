#!/bin/sh

# This list of commands was taken using the following command:
# rosdep install --reinstall --simulate --from-path ./src --ignore-src --rosdistro humble -r

sudo -H apt-get install ros-humble-moveit-ros-planning-interface
sudo -H apt-get install ros-humble-hardware-interface
sudo -H apt-get install ros-humble-controller-manager
sudo -H apt-get install ros-humble-serial-driver
sudo -H apt-get install ros-humble-v4l2-camera
sudo -H apt-get install python3-opencv
#[pip] Installation commands:
sudo -H pip3 install -U -I tensorflow
#[apt] Installation commands:
sudo -H apt-get install python3-scipy
sudo -H apt-get install cmake
#[pip] Installation commands:
sudo -H pip3 install -U -I dlib
#[apt] Installation commands:
sudo -H apt-get install python3-dev
sudo -H apt-get install python3-joblib
sudo -H apt-get install python3-numpy
sudo -H apt-get install ros-humble-cv-bridge
sudo -H apt-get install python3-pytest
sudo -H apt-get install python3-socketio
sudo -H apt-get install python3-aiohttp
#[pip] Installation commands:
sudo -H pip3 install -U -I keras
#[apt] Installation commands:
sudo -H apt-get install python3-pyaudio
#[pip] Installation commands:
sudo -H pip3 install -U -I sounddevice
#[apt] Installation commands:
sudo -H apt-get install ros-humble-joint-state-publisher-gui
sudo -H apt-get install ros-humble-rviz2
sudo -H apt-get install ros-humble-ros2-control
sudo -H apt-get install ros-humble-ros2-controllers
sudo -H apt-get install gazebo
sudo -H apt-get install ros-humble-moveit-ros-move-group
sudo -H apt-get install ros-humble-moveit-kinematics
sudo -H apt-get install ros-humble-moveit-planners
sudo -H apt-get install ros-humble-moveit-simple-controller-manager
sudo -H apt-get install ros-humble-joint-state-publisher
sudo -H apt-get install ros-humble-xacro
sudo -H apt-get install ros-humble-moveit-configs-utils
sudo -H apt-get install ros-humble-moveit-ros-visualization
sudo -H apt-get install ros-humble-moveit-ros-warehouse
sudo -H apt-get install ros-humble-moveit-setup-assistant
sudo -H apt-get install ros-humble-rviz-common
sudo -H apt-get install ros-humble-rviz-default-plugins
sudo -H apt-get install ros-humble-warehouse-ros-mongo