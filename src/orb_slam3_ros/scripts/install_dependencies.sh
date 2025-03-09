#!/bin/bash

echo "Installing required dependencies..."

# Install ROS navigation packages
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-ridgeback-description \
    ros-noetic-ridgeback-msgs \
    ros-noetic-ridgeback-control \
    ros-noetic-ridgeback-navigation \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    python3-pcl \
    python3-pip \
    portaudio19-dev \
    python3-pyaudio

# Install Python dependencies
pip3 install SpeechRecognition opencv-python

echo "Dependencies installed!"