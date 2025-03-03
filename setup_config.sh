#!/bin/bash

# Define the config directory
CONFIG_DIR="/home/praj/capstone_final/ridgeback_ws/src/orb_slam3_ros/config/D455"

# Create the directory if it doesn't exist
mkdir -p "$CONFIG_DIR"

# Create the D455.yaml file
CONFIG_FILE="$CONFIG_DIR/D455.yaml"
cat > "$CONFIG_FILE" << EOL
%YAML:1.0

# Camera calibration parameters (adjust these based on your actual camera calibration)
Camera.type: "PinHole"
Camera.fx: 386.947
Camera.fy: 386.947
Camera.cx: 320.958
Camera.cy: 240.288

# Distortion parameters
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# Camera resolution
Camera.width: 640
Camera.height: 480

# Camera frames per second 
Camera.fps: 30.0

# IR projector baseline times fx (depth scaling factor)
Camera.bf: 40.0

# Color order for the images (0: BGR, 1: RGB)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 40.0

# Depth map values factor
DepthMapFactor: 1000.0

# ORB Extractor parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# Viewer parameters
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
EOL

echo "Created D455.yaml at $CONFIG_FILE"

# Create symbolic link to the vocabulary file
VOC_DIR="/home/praj/capstone_final/ridgeback_ws/src/orb_slam3_ros/vocabulary"
mkdir -p "$VOC_DIR"

VOC_SOURCE="/home/praj/capstone_final/ridgeback_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
VOC_DEST="$VOC_DIR/ORBvoc.txt"

if [ -f "$VOC_SOURCE" ]; then
    ln -sf "$VOC_SOURCE" "$VOC_DEST"
    echo "Created symbolic link to vocabulary at $VOC_DEST"
else
    echo "Warning: ORBvoc.txt not found at $VOC_SOURCE"
fi

echo "Configuration setup complete."