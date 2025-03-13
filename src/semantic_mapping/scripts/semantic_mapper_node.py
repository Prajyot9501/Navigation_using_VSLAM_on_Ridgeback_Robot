#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import sqlite3
import os
import json
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from semantic_mapping.srv import QueryObject
from visualization_msgs.msg import Marker, MarkerArray
from yolov8_ros.msg import Detections2D, Detection2D

class SemanticMapper:
    def __init__(self):
        rospy.init_node('semantic_mapper', anonymous=True)
        
        # Parameters
        self.db_path = rospy.get_param('~db_path', os.path.expanduser('~/semantic_map.db'))
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.7)
        self.min_detections = rospy.get_param('~min_detections', 3)  # Minimum detections to confirm an object
        self.max_distance = rospy.get_param('~max_distance', 0.5)    # Maximum distance (meters) to consider as the same object
        self.camera_frame = rospy.get_param('~camera_frame', 'ORB_SLAM3')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # Initialize database
        self.init_database()
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.camera_info = None
        self.depth_image = None
        self.rgb_image = None
        
        # Store recent detections for temporal filtering
        self.recent_detections = {}  # class_name -> [detections]
        
        # Publishers
        self.marker_pub = rospy.Publisher('/semantic_map/markers', MarkerArray, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/yolo/detections', Detections2D, self.detections_callback)
        
        # Services
        self.query_service = rospy.Service('/semantic_map/query', QueryObject, self.handle_query)
        
        # Timer for publishing markers
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        
        rospy.loginfo("Semantic mapper initialized with database: %s", self.db_path)
        
    def init_database(self):
        """Initialize SQLite database for semantic objects"""
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute('''
        CREATE TABLE IF NOT EXISTS objects
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
         class_name TEXT,
         instance_id INTEGER,
         x REAL,
         y REAL,
         z REAL,
         confidence REAL,
         detection_count INTEGER,
         last_seen REAL,
         first_seen REAL)
        ''')
        conn.commit()
        conn.close()
    
    def camera_info_callback(self, msg):
        """Store camera info for projection calculations"""
        self.camera_info = msg
    
    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def rgb_callback(self, msg):
        """Store the latest RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
    
    def detections_callback(self, msg):
        """Process object detections and add to database"""
        if self.camera_info is None or self.depth_image is None:
            rospy.logwarn("Waiting for camera info and depth image...")
            return
        
        # Process each detection
        for detection in msg.detections:
            if detection.confidence < self.confidence_threshold:
                continue
            
            # Get center pixel in bounding box
            center_x = int((detection.x_min + detection.x_max) / 2)
            center_y = int((detection.y_min + detection.y_max) / 2)
            
            # Safety check for image bounds
            if (center_x < 0 or center_x >= self.depth_image.shape[1] or
                center_y < 0 or center_y >= self.depth_image.shape[0]):
                continue
            
            # Get depth value at center
            # For better accuracy, we could use an average depth within the bounding box
            depth = self.depth_image[center_y, center_x] / 1000.0  # Convert mm to meters
            
            # Skip invalid depth values
            if depth <= 0 or np.isnan(depth) or np.isinf(depth):
                continue
            
            # Get camera intrinsics
            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            cx = self.camera_info.K[2]
            cy = self.camera_info.K[5]
            
            # Calculate 3D point in camera coordinates
            x_camera = (center_x - cx) * depth / fx
            y_camera = (center_y - cy) * depth / fy
            z_camera = depth
            
            # Transform point to world coordinates
            p_camera = PointStamped()
            p_camera.header.frame_id = self.camera_frame
            p_camera.header.stamp = msg.header.stamp
            p_camera.point.x = z_camera  # ORB-SLAM3 uses Z forward, X right, Y down
            p_camera.point.y = -x_camera
            p_camera.point.z = -y_camera
            
            try:
                # Use latest available transform instead of message timestamp
                p_camera.header.stamp = rospy.Time(0)  # this requests the latest transform
                p_world = self.tf_buffer.transform(p_camera, self.world_frame)
                
                # Add to recent detections for temporal filtering
                class_name = detection.class_name
                if class_name not in self.recent_detections:
                    self.recent_detections[class_name] = []
                
                # Add detection with timestamp and position
                self.recent_detections[class_name].append({
                    'timestamp': rospy.Time.now().to_sec(),
                    'position': (p_world.point.x, p_world.point.y, p_world.point.z),
                    'confidence': detection.confidence
                })
                
                # Process recent detections to find stable objects
                self.process_detections(class_name)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")
    
    def process_detections(self, class_name):
        """Process temporal detections to find stable objects"""
        detections = self.recent_detections[class_name]
        
        # Remove detections older than 5 seconds
        current_time = rospy.Time.now().to_sec()
        detections = [d for d in detections if current_time - d['timestamp'] < 5.0]
        self.recent_detections[class_name] = detections
        
        # If we have enough detections, try to cluster them
        if len(detections) >= self.min_detections:
            # Simple clustering algorithm
            clusters = []
            
            for detection in detections:
                pos = detection['position']
                assigned = False
                
                # Try to assign to existing cluster
                for cluster in clusters:
                    cluster_pos = cluster['center']
                    dist = np.sqrt((pos[0] - cluster_pos[0])**2 + 
                                   (pos[1] - cluster_pos[1])**2 + 
                                   (pos[2] - cluster_pos[2])**2)
                    
                    if dist < self.max_distance:
                        # Add to cluster
                        cluster['detections'].append(detection)
                        
                        # Update center (weighted average)
                        total_conf = sum(d['confidence'] for d in cluster['detections'])
                        x = sum(d['position'][0] * d['confidence'] for d in cluster['detections']) / total_conf
                        y = sum(d['position'][1] * d['confidence'] for d in cluster['detections']) / total_conf
                        z = sum(d['position'][2] * d['confidence'] for d in cluster['detections']) / total_conf
                        
                        cluster['center'] = (x, y, z)
                        cluster['confidence'] = total_conf / len(cluster['detections'])
                        assigned = True
                        break
                
                if not assigned:
                    # Create new cluster
                    clusters.append({
                        'center': pos,
                        'detections': [detection],
                        'confidence': detection['confidence']
                    })
            
            # Check clusters with enough detections
            for cluster in clusters:
                if len(cluster['detections']) >= self.min_detections:
                    self.add_object_to_database(class_name, cluster)
    
    def add_object_to_database(self, class_name, cluster):
        """Add or update an object in the database"""
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        
        pos = cluster['center']
        
        # Check if object already exists within a distance threshold
        c.execute('''
        SELECT id, detection_count, x, y, z FROM objects 
        WHERE class_name = ? 
        ''', (class_name,))
        
        objects = c.fetchall()
        object_id = None
        min_dist = float('inf')
        
        for obj in objects:
            db_id, count, db_x, db_y, db_z = obj
            dist = np.sqrt((pos[0] - db_x)**2 + (pos[1] - db_y)**2 + (pos[2] - db_z)**2)
            
            if dist < self.max_distance and dist < min_dist:
                object_id = db_id
                min_dist = dist
        
        # Get current time
        current_time = rospy.Time.now().to_sec()
        
        if object_id is not None:
            # Update existing object
            c.execute('''
            UPDATE objects SET 
            x = ?, y = ?, z = ?, 
            confidence = ?, 
            detection_count = detection_count + 1,
            last_seen = ?
            WHERE id = ?
            ''', (pos[0], pos[1], pos[2], 
                 cluster['confidence'], 
                 current_time, object_id))
        else:
            # Insert new object
            c.execute('''
            INSERT INTO objects 
            (class_name, instance_id, x, y, z, confidence, detection_count, last_seen, first_seen)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (class_name, 
                 self.get_next_instance_id(c, class_name),
                 pos[0], pos[1], pos[2], 
                 cluster['confidence'], 
                 len(cluster['detections']),
                 current_time, current_time))
        
        conn.commit()
        conn.close()
    
    def get_next_instance_id(self, cursor, class_name):
        """Get the next available instance ID for a class"""
        cursor.execute('''
        SELECT MAX(instance_id) FROM objects WHERE class_name = ?
        ''', (class_name,))
        
        result = cursor.fetchone()[0]
        if result is None:
            return 1
        else:
            return result + 1
    
    def publish_markers(self, event):
        """Publish visualization markers for all objects in database"""
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        
        # Get all objects from database
        c.execute('''
        SELECT id, class_name, instance_id, x, y, z, confidence FROM objects
        ''')
        
        objects = c.fetchall()
        conn.close()
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Get current time for marker lifetime
        now = rospy.Time.now()
        
        # Create a marker for each object
        for i, obj in enumerate(objects):
            obj_id, class_name, instance_id, x, y, z, confidence = obj
            
            # Create text marker for object label
            text_marker = Marker()
            text_marker.header.frame_id = self.world_frame
            text_marker.header.stamp = now
            text_marker.ns = "object_labels"
            text_marker.id = obj_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.3  # Above the object
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.2  # Text height
            
            # Set color based on confidence
            text_marker.color.r = 1.0 - confidence
            text_marker.color.g = confidence
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{class_name}"
            text_marker.lifetime = rospy.Duration(1.5)  # Marker will last for 1.5 seconds
            
            # Create sphere marker for object position
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.world_frame
            sphere_marker.header.stamp = now
            sphere_marker.ns = "object_positions"
            sphere_marker.id = obj_id
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            sphere_marker.pose.position.x = x
            sphere_marker.pose.position.y = y
            sphere_marker.pose.position.z = z
            sphere_marker.pose.orientation.w = 1.0
            
            sphere_marker.scale.x = 0.2
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            
            # Set color based on object class (simple hash function)
            hue = hash(class_name) % 360 / 360.0
            self.hsv_to_rgb(hue, 1.0, 1.0, sphere_marker.color)
            sphere_marker.color.a = 0.8
            
            sphere_marker.lifetime = rospy.Duration(1.5)
            
            marker_array.markers.append(text_marker)
            marker_array.markers.append(sphere_marker)
        
        # Publish marker array
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def hsv_to_rgb(self, h, s, v, color_msg):
        """Convert HSV to RGB and set in color message"""
        if s == 0.0:
            color_msg.r = v
            color_msg.g = v
            color_msg.b = v
            return
        
        i = int(h * 6.0)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i %= 6
        
        if i == 0:
            color_msg.r = v
            color_msg.g = t
            color_msg.b = p
        elif i == 1:
            color_msg.r = q
            color_msg.g = v
            color_msg.b = p
        elif i == 2:
            color_msg.r = p
            color_msg.g = v
            color_msg.b = t
        elif i == 3:
            color_msg.r = p
            color_msg.g = q
            color_msg.b = v
        elif i == 4:
            color_msg.r = t
            color_msg.g = p
            color_msg.b = v
        elif i == 5:
            color_msg.r = v
            color_msg.g = p
            color_msg.b = q
    
    def handle_query(self, req):
        """Handle query service requests"""
        # Parse query string (simple format: "class_name" or "class_name instance_id")
        query = req.query.strip()
        parts = query.split()
        
        if len(parts) == 1:
            # Query for any instance of a class
            class_name = parts[0]
            instance_id = None
        elif len(parts) == 2:
            # Query for specific instance of a class
            class_name = parts[0]
            try:
                instance_id = int(parts[1])
            except ValueError:
                return "Error: Invalid instance ID. Format should be: 'class_name [instance_id]'"
        else:
            return "Error: Invalid query format. Use 'class_name' or 'class_name instance_id'"
        
        # Query database
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        
        if instance_id is None:
            # Find the most recently seen instance of the class
            c.execute('''
            SELECT id, instance_id, x, y, z, last_seen 
            FROM objects 
            WHERE class_name = ? 
            ORDER BY last_seen DESC
            LIMIT 1
            ''', (class_name,))
        else:
            # Find the specific instance
            c.execute('''
            SELECT id, instance_id, x, y, z, last_seen 
            FROM objects 
            WHERE class_name = ? AND instance_id = ?
            ''', (class_name, instance_id))
        
        result = c.fetchone()
        conn.close()
        
        if result is None:
            return f"No object found: {query}"
        
        # Return object information
        obj_id, inst_id, x, y, z, last_seen = result
        time_ago = rospy.Time.now().to_sec() - last_seen
        
        return json.dumps({
            "class_name": class_name,
            "instance_id": inst_id,
            "position": {
                "x": x,
                "y": y,
                "z": z
            },
            "time_since_last_seen": time_ago
        })

if __name__ == '__main__':
    try:
        mapper = SemanticMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass