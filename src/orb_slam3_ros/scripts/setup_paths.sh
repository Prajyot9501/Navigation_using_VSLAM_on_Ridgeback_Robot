#!/bin/bash
# setup_paths.sh - Script to set up library paths for ORB-SLAM3

if [ $# -lt 1 ]; then
  echo "Usage: $0 <orb_slam3_lib_path>"
  exit 1
fi

ORB_SLAM3_LIB_PATH="$1"

echo "Setting up environment for ORB-SLAM3"
echo "ORB_SLAM3 library path: $ORB_SLAM3_LIB_PATH"

# Check if the library exists
if [ ! -f "$ORB_SLAM3_LIB_PATH/libORB_SLAM3.so" ]; then
  echo "ERROR: libORB_SLAM3.so not found at $ORB_SLAM3_LIB_PATH"
  echo "Please check your ORB_SLAM3 installation"
  exit 1
else
  echo "Found libORB_SLAM3.so at $ORB_SLAM3_LIB_PATH"
fi

# Set LD_LIBRARY_PATH to include ORB_SLAM3/lib
export LD_LIBRARY_PATH="$ORB_SLAM3_LIB_PATH:$LD_LIBRARY_PATH"
echo "Updated LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

# Return success
exit 0