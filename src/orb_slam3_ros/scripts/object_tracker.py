#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
import tf
import message_filters

class ObjectTracker:
    def __init__(self):
        rospy.init_node('object_tracker', anonymous=True)
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.object_classes = {
            0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck",
            # Add more classes as needed
        }
        
        # Object storage
        self.tracked_objects = {}  # class_id -> [x, y, z, confidence, timestamp]
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Publishers
        self.object_markers_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=1)
        self.target_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # Initialize synchronized subscribers
        rgb_sub = message_filters.Subscriber('/device_0/sensor_1/Color_0/image/data', Image)
        depth_sub = message_filters.Subscriber('/device_0/sensor_0/Depth_0/image/data', Image)
        
        # The ApproximateTimeSynchronizer synchronizes incoming messages by their timestamps
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)
        
        # Voice command subscriber (simple string command)
        rospy.Subscriber('/voice_command', String, self.voice_command_callback)
        
        # Start publishing object markers
        rospy.Timer(rospy.Duration(1.0), self.publish_object_markers)
        
        rospy.spin()
    
    def image_callback(self, rgb_msg, depth_msg):
        # Convert to OpenCV images
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")  # 16-bit depth
        
        # Get camera intrinsics (should match your camera calibration in the YAML file)
        fx = 386.947
        fy = 386.947
        cx = 320.958
        cy = 240.288
        depth_scale = 1000.0  # Scale factor for depth values
        
        # Run YOLOv8 detection (you already have this in your detector node)
        # This is a placeholder - in practice you'd subscribe to detections
        detections = self.detect_objects(rgb_image)
        
        for obj_class, bbox, conf in detections:
            if conf < self.confidence_threshold:
                continue
                
            # Get center of bounding box
            x_center = (bbox[0] + bbox[2]) / 2
            y_center = (bbox[1] + bbox[3]) / 2
            
            # Get depth at the center point
            depth_value = depth_image[int(y_center), int(x_center)] / depth_scale  # Convert to meters
            
            if depth_value <= 0 or depth_value > 10:  # Ignore invalid depths
                continue
                
            # Convert from image to camera coordinates
            x = (x_center - cx) * depth_value / fx
            y = (y_center - cy) * depth_value / fy
            z = depth_value
            
            # Transform from camera to world coordinates
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'camera_link', rospy.Time(0))
                
                # Apply transformation
                point_camera = np.array([x, y, z, 1.0])
                
                # Create transformation matrix
                transform = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(trans),
                    tf.transformations.quaternion_matrix(rot)
                )
                
                # Apply transformation
                point_world = np.dot(transform, point_camera)
                
                # Update tracked objects dictionary
                self.tracked_objects[obj_class] = {
                    'position': [point_world[0], point_world[1], point_world[2]],
                    'confidence': conf,
                    'timestamp': rospy.Time.now()
                }
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")
    
    def voice_command_callback(self, msg):
        # Simple voice command processing
        command = msg.data.lower()
        
        # Check if command is to go to an object
        if "go to" in command:
            # Extract object name
            for obj_class, obj_name in self.object_classes.items():
                if obj_name in command:
                    self.navigate_to_object(obj_class)
                    break
    
    def navigate_to_object(self, obj_class):
        if obj_class in self.tracked_objects:
            obj_data = self.tracked_objects[obj_class]
            
            # Create navigation goal
            goal = PoseStamped()
            goal.header.frame_id = "world"
            goal.header.stamp = rospy.Time.now()
            
            # Set position (offset a bit for safety)
            pos = obj_data['position']
            goal.pose.position.x = pos[0] - 0.5  # Offset 0.5m from object
            goal.pose.position.y = pos[1]
            goal.pose.position.z = 0.0  # Keep on ground plane
            
            # Orient towards the object
            dx = pos[0] - goal.pose.position.x
            dy = pos[1] - goal.pose.position.y
            yaw = np.arctan2(dy, dx)
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            
            # Publish goal
            self.target_pub.publish(goal)
            rospy.loginfo(f"Navigating to {self.object_classes[obj_class]} at position {pos}")
    
    def publish_object_markers(self, event=None):
        marker_array = MarkerArray()
        
        # Clean old objects (older than 10 seconds)
        current_time = rospy.Time.now()
        keys_to_remove = []
        
        for obj_class, obj_data in self.tracked_objects.items():
            time_diff = (current_time - obj_data['timestamp']).to_sec()
            
            if time_diff > 10.0:
                keys_to_remove.append(obj_class)
        
        for key in keys_to_remove:
            del self.tracked_objects[key]
        
        # Create markers for remaining objects
        for i, (obj_class, obj_data) in enumerate(self.tracked_objects.items()):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            pos = obj_data['position']
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Set color (different for each class)
            hue = (obj_class * 30) % 180  # Cycle through different hues
            color = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
            marker.color.r = color[2] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[0] / 255.0
            marker.color.a = 0.8
            
            # Text marker for object class
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "object_labels"
            text_marker.id = i + 100  # Offset IDs for text markers
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = marker.pose.position
            text_marker.pose.position.z += 0.5  # Place text above the cube
            text_marker.text = self.object_classes.get(obj_class, f"Unknown {obj_class}")
            text_marker.scale.z = 0.3  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)
        
        # Publish markers
        self.object_markers_pub.publish(marker_array)
    
    def detect_objects(self, image):
        """Placeholder for object detection. In practice, subscribe to YOLOv8 detections."""
        # This should return a list of (class_id, bbox, confidence)
        # where bbox is [x1, y1, x2, y2]
        return []  # Empty list for now - you'll actually get this from your YOLO node

if __name__ == '__main__':
    try:
        ObjectTracker()
    except rospy.ROSInterruptException:
        pass