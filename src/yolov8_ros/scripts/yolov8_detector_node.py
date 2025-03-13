#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from ultralytics import YOLO
import torch

# Import custom message
from yolov8_ros.msg import Detection2D, Detections2D

class YOLOv8Detector:
    def __init__(self):
        rospy.init_node('yolov8_detector', anonymous=True)
        
        # Parameters
        self.model_path = rospy.get_param('~model_path', 'yolov8n-seg.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.dynamic_classes = rospy.get_param('~dynamic_classes', [0, 1, 2, 3, 5, 7])  # COCO classes for person, bicycle, car, motorcycle, bus, truck
        
        # Load YOLOv8 model
        self.model = YOLO(self.model_path, verbose=False)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.mask_pub = rospy.Publisher('/dynamic_mask', Image, queue_size=1)
        self.vis_pub = rospy.Publisher('/detection_visualization', Image, queue_size=1)
        self.detections_pub = rospy.Publisher('/yolo/detections', Detections2D, queue_size=1)
        
        # Camera info subscriber for potential future 3D projection
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        rospy.loginfo("YOLOv8 detector initialized with model: %s", self.model_path)
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Create mask for dynamic objects (initialized as all zeros = static)
            dynamic_mask = np.zeros((height, width), dtype=np.uint8)
            
            # Run YOLOv8 detection
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            # Visualization image
            vis_image = cv_image.copy()
            
            # Create detection message
            detection_msg = Detections2D()
            detection_msg.header = msg.header
            
            # Process detection results
            for r in results:
                boxes = r.boxes
                masks = r.masks if hasattr(r, 'masks') else None
                
                for i, box in enumerate(boxes):
                    # Get detection info
                    cls_id = int(box.cls.item())
                    conf = float(box.conf.item())
                    x1, y1, x2, y2 = map(float, box.xyxy[0])
                    class_name = self.model.names[cls_id]
                    
                    # Create Detection2D message
                    det = Detection2D()
                    det.class_name = class_name
                    det.class_id = cls_id
                    det.confidence = conf
                    det.x_min = x1
                    det.y_min = y1
                    det.x_max = x2
                    det.y_max = y2
                    
                    # Add to detections message
                    detection_msg.detections.append(det)
                    
                    # Process segmentation masks if available
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        # Check if object is dynamic
                        if cls_id in self.dynamic_classes:
                            # Convert mask to numpy array and resize to image dimensions
                            mask_array = mask.data.cpu().numpy().squeeze()
                            # Resize mask to original image size
                            mask_array = cv2.resize(mask_array, (width, height))
                            # Threshold mask to binary (0 or 255)
                            _, mask_binary = cv2.threshold(mask_array, 0.5, 255, cv2.THRESH_BINARY)
                            mask_binary = mask_binary.astype(np.uint8)
                            
                            # Add to dynamic mask
                            dynamic_mask = cv2.bitwise_or(dynamic_mask, mask_binary)
                            
                            # Draw on visualization image
                            color = (0, 0, 255)  # Red for dynamic objects
                            vis_image = self.draw_mask_overlay(vis_image, mask_binary, color, 0.5)
                    
                    # Draw bounding box and label (for all objects, not just dynamic)
                    is_dynamic = cls_id in self.dynamic_classes
                    color = (0, 0, 255) if is_dynamic else (0, 255, 0)  # Red for dynamic, green for static
                    
                    cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(vis_image, f"{class_name} {conf:.2f}", 
                                (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish detection message
            self.detections_pub.publish(detection_msg)
            
            # Publish dynamic mask
            mask_msg = self.bridge.cv2_to_imgmsg(dynamic_mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def draw_mask_overlay(self, image, mask, color, alpha):
        """Draw semi-transparent mask overlay on image"""
        mask_color = np.zeros_like(image, dtype=np.uint8)
        mask_color[mask > 0] = color
        return cv2.addWeighted(mask_color, alpha, image, 1 - alpha, 0)

if __name__ == '__main__':
    try:
        detector = YOLOv8Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass