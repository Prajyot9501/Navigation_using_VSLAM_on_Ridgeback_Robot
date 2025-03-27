# Autonomous Mobile Robot Navigation System for Indoor Assistant Robot using Visual SLAM


## :dart: Mission
To get the Clearpath Robotics Ridgeback to navigate on its own with a map and stored object locations as per users’ instructions 

This project enables autonomous navigation for an indoor mobile robot, using advanced computer vision and mapping techniques. The robot—powered by Clearpath Robotics’ Ridgeback platform is equipped with Intel RealSense D455 and utilizes ORB-SLAM3, YOLOv8, and Octomap to map, detect, and navigate environments.

**Our Hero:** Ridgeback, the omnidirectional wonder-robot <br>
**Sidekick:** Intel RealSense D455 camera (Ridgeback's eyes) <br>
**Quest:** Teaching Ridgeback to map rooms and navigate without bumping into things <br>
**Secret Weapons:** <br> 
  + ROS Robot Operating System) : For integration and communication between modules
  + Visual SLAM technology (photographic memory) : Visual SLAM system for real-time 3D map creation and localization
  + YOLO v8 object detection (identifying objects like a pro)
  + Octomap (3D memory of the world)
##  System Components
### Hardware
- Clearpath Robotics Ridgeback
-  Intel RealSense D455 camera
  
### Software
- ROS (Robot Operating System) integration
- Visual SLAM using ORB-SLAM3
- Object detection with YOLOv8
- 3D mapping via OctoMap
- Sensor data processing from Intel RealSense D455
- Ridgeback robot motion planning and control
- TF transformation management for accurate spatial localization
- RViz visualization for real-time monitoring
- Custom ROS wrapper for YOLOv8 and ORB-SLAM3 integration
- Object context storage in a semantic database

## Installation
### Prerequisites
- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)
- [Intel RealSense SDK 2.0](https://www.intelrealsense.com/sdk-2/)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [YOLOv8 (via Ultralytics)](https://huggingface.co/Ultralytics/YOLOv8)
- [OpenCV 4.4+](https://opencv.org/blog/opencv-4-4-0/)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [Eigen3](https://github.com/stevenlovegrove/Pangolin)
- Python 3.8+
- C++11 or C++0x Compiler

## 🛠️ Implementation Steps
### 1. Data Collection
-Recorded .bag files using RealSense D455 <br>
-Validated RGB and Depth streams via rosbag and Realsense SDK <br>
<img src="https://github.com/user-attachments/assets/c87fb34e-bc74-4d82-8aaf-463f712199f5" width="300">

### 2. Visual SLAM with ORB-SLAM3
-Installed required dependencies: OpenCV 4.4, Pangolin, OpenGL <br>
-Ran ORB-SLAM3 with RGB-D configuration <br>
-Handled TF orientation and loop closure for accurate mapping <br>
<img src="https://github.com/user-attachments/assets/5621f89e-2a8d-4642-ba7f-a83ab47fdb7a" width="300">

### 3. Object Detection with YOLOv8
-Created a ROS wrapper for YOLOv8 integration <br>
-Stored object detections with depth info and avoided duplicates <br>
-Mapped 3D coordinates of identified objects<br>
<img src="https://github.com/user-attachments/assets/abf9e630-e516-4bdf-97bc-8e5ed25a51c8" width="300">

### 4. Navigation and Control
-Spawned Ridgeback in RViz for simulation <br>
-Developed custom control node <br>
-Integrated map and object locations for path planning
![nav](https://github.com/user-attachments/assets/4277e194-56ec-4d80-83ed-95ce21b7d2cf)


## Setup Instructions
#### Clone the repository

```
git clone https://github.com/your-username/Nursery_Robot_Panda.git
cd Nursery_Robot_Panda/catkin_ws
```
#### Build ROS Workspace Isolated
```
catkin_make_isolated --install <br>
source install_isolated/setup.bash
```
#### Create map and semantic_db.db file with object detected data from your .bag file with camera captured data
```
roslaunch ridgeback_semantic_nav map_generation_from_bag.launch bag_file:={path_to_your_bag_file}.bag
```
>[!NOTE]
>Map is generated and saved as my_map.pgm and my_map.yaml

> [!TIP]
> Open 3 separate terminals to run next 3 commands

#### Terminal 1: ***Run roscore in order for ROS nodes to communicate***
```
roscore
```
#### Terminal 2: ***Open RViz and see map with detected objects with robot in it***
```
roslaunch ridgeback_semantic_nav absolute_path_navigation.launch
```
#### Terminal 3: ***Run navigation command by running python script, asks robot to move to specified object***
```
rosrun ridgeback_semantic_nav goto_object.py {object_name}
```

## 🧑‍🔬 Developed By
Prajyot Patil <br>
Mentored by Dr. Maria Kyrarini <br>
Santa Clara University
