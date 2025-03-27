# Autonomous Mobile Robot Navigation System for Indoor Assistant Robot using Visual SLAM


## :dart: Mission
To get the Clearpath Robotics Ridgeback to navigate on its own with a map and stored object locations as per users’ instructions 

This project enables autonomous navigation for an indoor mobile robot, using advanced computer vision and mapping techniques. The robot—powered by Clearpath Robotics’ Ridgeback platform—is equipped with Intel RealSense D455 and utilizes ORB-SLAM3, YOLOv8, and Octomap to map, detect, and navigate environments.

**Our Hero:** Ridgeback, the omnidirectional wonder-robot <br>
**Sidekick:** Intel RealSense D455 camera (Ridgeback's eyes) <br>
**Quest:** Teaching Ridgeback to map rooms and navigate without bumping into things <br>
**Secret Weapons:** <br> 
  + ROS Robot Operating System) : For integration and communication between modules
  + Visual SLAM technology (photographic memory) : Visual SLAM system for real-time 3D map creation and localization
  + YOLO v8 object detection (identifying objects like a pro)
  + Octomap (3D memory of the world)

## 🛠️ Implementation Steps
### 1. Data Collection
Recorded .bag files using RealSense D455 <br>
Validated RGB and Depth streams via rosbag and Realsense SDK <br>

### 2. Visual SLAM with ORB-SLAM3
Installed required dependencies: OpenCV 4.4, Pangolin, OpenGL <br>
Ran ORB-SLAM3 with RGB-D configuration <br>
Handled TF orientation and loop closure for accurate mapping

### 3. Object Detection with YOLOv8
Created a ROS wrapper for YOLOv8 integration <br>
Stored object detections with depth info and avoided duplicates <br>
Mapped 3D coordinates of identified objects

### 4. Navigation and Control
Spawned Ridgeback in RViz for simulation <br>
Developed custom control node <br>
Integrated map and object locations for path planning
