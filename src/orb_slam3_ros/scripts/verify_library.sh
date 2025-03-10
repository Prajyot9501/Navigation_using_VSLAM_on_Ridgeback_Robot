#!/bin/bash
# verify_library.sh - Script to verify ORB-SLAM3 library exists

if [ $# -lt 1 ]; then
  echo "Usage: $0 <orb_slam3_lib_path>"
  exit 1
fi

ORB_SLAM3_LIB_PATH="$1"

echo "Verifying ORB-SLAM3 library installation..."
echo "Checking path: $ORB_SLAM3_LIB_PATH"

# Check if the library exists
if [ ! -f "$ORB_SLAM3_LIB_PATH/libORB_SLAM3.so" ]; then
  echo "ERROR: libORB_SLAM3.so not found at $ORB_SLAM3_LIB_PATH"
  echo "Please update the path in the launch file or install ORB-SLAM3 properly"
  exit 1
else
  echo "SUCCESS: Found libORB_SLAM3.so at $ORB_SLAM3_LIB_PATH"
fi

# Also check dependencies with ldd
if command -v ldd &> /dev/null; then
  echo "Checking library dependencies..."
  ldd "$ORB_SLAM3_LIB_PATH/libORB_SLAM3.so" | grep "not found" || echo "All dependencies satisfied"
else
  echo "ldd command not found, skipping dependency check"
fi

# Return success
exit 0