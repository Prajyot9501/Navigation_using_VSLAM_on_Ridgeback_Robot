#!/bin/bash
echo "Testing ORB_SLAM3 library loading"
export LD_LIBRARY_PATH=~/capstone_final/ridgeback_ws/src/ORB_SLAM3/lib:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "Checking if library exists:"
ls -la ~/capstone_final/ridgeback_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so || echo "Library not found!"
echo "Attempting to load library with ldd:"
ldd ~/capstone_final/ridgeback_ws/install_isolated/lib/orb_slam3_ros/orb_slam3_ros_yolo_rgbd | grep ORB_SLAM3
echo "Test complete"