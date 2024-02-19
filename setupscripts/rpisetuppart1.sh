#!/bin/bash

echo "enabling ssh"
sudo systemctl enable ssh
sudo systemctl start ssh

echo "Installing Software"
sudo apt update
sudo apt install python3 python3-pip git
sudo apt install python-is-python3
sudo apt install vlc python3-opencv tmux
pip3 install matplotlib
pip3 install transitions scikit-learn

echo "Installing ROS"
sudo sh -c 'echo "deb <http://packages.ros.org/ros/ubuntu> $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s <https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc> | sudo apt-key add -
sudo apt install build-essential arduino arduino-mk
sudo apt install ros-noetic-rosserial ros-noetic- rosserial-arduino

echo "-part 1 finished-"
