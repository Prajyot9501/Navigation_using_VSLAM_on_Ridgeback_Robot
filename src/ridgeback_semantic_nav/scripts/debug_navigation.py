#!/usr/bin/env python3

import rospy
import json
import sys
from semantic_mapping.srv import QueryObject
from geometry_msgs.msg import PoseStamped

def debug_semantic_service():
    """Debug the semantic map query service"""
    rospy.init_node('debug_semantic_navigation', anonymous=True)
    
    # Check if the service exists
    service_name = '/semantic_map/query'
    rospy.loginfo(f"Checking service: {service_name}")
    
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        rospy.loginfo(f"Service {service_name} is available!")
        
        # Try to call the service
        query_service = rospy.ServiceProxy(service_name, QueryObject)
        response = query_service(query="chair")
        
        rospy.loginfo(f"Service response: {response.response}")
        
        try:
            # Try to parse the response as JSON
            data = json.loads(response.response)
            rospy.loginfo(f"Position data: x={data['position']['x']}, y={data['position']['y']}, z={data['position']['z']}")
            
            # Try to publish this as a direct move_base goal
            rospy.loginfo("Publishing direct move_base goal...")
            pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
            
            # Create a goal message
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            
            # Set the position
            goal.pose.position.x = data['position']['x']
            goal.pose.position.y = data['position']['y']
            goal.pose.position.z = 0.0
            
            # Set orientation (identity quaternion)
            goal.pose.orientation.w = 1.0
            
            # Sleep to allow publisher to connect
            rospy.sleep(1.0)
            
            # Publish the goal
            pub.publish(goal)
            rospy.loginfo(f"Published goal at x={goal.pose.position.x}, y={goal.pose.position.y}")
            
        except json.JSONDecodeError:
            rospy.logwarn("Response is not in JSON format")
        
    except rospy.ROSException as e:
        rospy.logerr(f"Service not available: {e}")
    
    # Check TF tree
    rospy.loginfo("Checking TF tree status...")
    
    # List of important TF frames to check
    important_frames = ['world', 'map', 'odom', 'base_link']
    
    rospy.loginfo("Important TF frames to check:")
    for frame in important_frames:
        rospy.loginfo(f"  - {frame}")
    
    rospy.loginfo("Run 'rosrun tf tf_monitor' in another terminal to check these frames")
    rospy.loginfo("Run 'rosrun tf view_frames' to generate a PDF of the TF tree")
    
if __name__ == '__main__':
    try:
        debug_semantic_service()
    except rospy.ROSInterruptException:
        pass