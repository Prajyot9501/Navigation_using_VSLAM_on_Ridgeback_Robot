#!/usr/bin/env python3

import rospy
import json
import sys
import argparse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from semantic_mapping.srv import QueryObject
from visualization_msgs.msg import Marker

class ObjectNavigator:
    def __init__(self):
        rospy.init_node('object_navigator', anonymous=True)
        
        # Parameters
        self.approach_distance = rospy.get_param('~approach_distance', 0.75)  # meters
        self.frame_id = rospy.get_param('~frame_id', 'map')
        
        # Set up visualization marker
        self.marker_pub = rospy.Publisher('/object_target', Marker, queue_size=1)
        
        # Connect to move_base action server
        rospy.loginfo("Connecting to move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Wait for the action server with a timeout
        server_available = self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0))
        if server_available:
            rospy.loginfo("Connected to move_base action server")
        else:
            rospy.logwarn("Could not connect to move_base action server, will use direct topic instead")
            # Fallback to simple topic-based navigation
            self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    def navigate_to_object(self, object_query):
        """Query the semantic map and navigate to the specified object"""
        rospy.loginfo("Navigating to object: %s", object_query)
        
        # Query the semantic map for the object
        try:
            service_name = '/semantic_map/query'
            rospy.loginfo("Waiting for semantic map service...")
            rospy.wait_for_service(service_name, timeout=5.0)
            
            query_service = rospy.ServiceProxy(service_name, QueryObject)
            response = query_service(query=object_query)
            
            rospy.loginfo("Semantic map response: %s", response.response)
            
            try:
                # Parse the response
                object_data = json.loads(response.response)
                
                # Extract object position
                position = object_data['position']
                x, y, z = position['x'], position['y'], position['z']
                
                rospy.loginfo("Found %s at position: (%.2f, %.2f, %.2f)", 
                             object_data['class_name'], x, y, z)
                
                # Publish a marker at the target location
                self.publish_target_marker(x, y, z, object_data['class_name'])
                
                # Send the navigation goal
                return self.send_navigation_goal(x, y, z)
                
            except json.JSONDecodeError:
                rospy.logerr("Failed to parse response as JSON: %s", response.response)
                return False
                
        except rospy.ROSException as e:
            rospy.logerr("Service error: %s", str(e))
            return False
    
    def publish_target_marker(self, x, y, z, object_name):
        """Publish a visualization marker at the target location"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navigation_targets"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        rospy.loginfo("Object 3D position: (%.2f, %.2f, %.2f)", x, y, z)
        rospy.loginfo("Sending navigation goal to 2D position: (%.2f, %.2f, 0.0)", x, y)
        
        # Set scale and color (bright green)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Set text label
        marker.text = f"Target: {object_name}"
        
        # Set lifetime
        marker.lifetime = rospy.Duration(0)  # 0 means forever
        
        # Publish the marker
        self.marker_pub.publish(marker)
        rospy.loginfo("Published target marker at (%.2f, %.2f, %.2f)", x, y, z)
    
    def send_navigation_goal(self, x, y, z):
        """Send a navigation goal to move_base"""
        # Create a goal message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        
        # Set the position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0  # Keep the robot on the ground
        
        # Set orientation (identity quaternion)
        goal_pose.pose.orientation.w = 1.0
        
        # Try action client approach first
        if hasattr(self, 'move_base_client') and self.move_base_client is not None:
            # Convert to MoveBaseGoal
            goal = MoveBaseGoal()
            goal.target_pose = goal_pose
            
            rospy.loginfo("Sending navigation goal to (%.2f, %.2f, %.2f) via action client", x, y, z)
            self.move_base_client.send_goal(goal)
            
            # Wait for result with timeout
            rospy.loginfo("Waiting for navigation to complete...")
            success = self.move_base_client.wait_for_result(rospy.Duration(60.0))
            
            if success:
                state = self.move_base_client.get_state()
                rospy.loginfo("Navigation finished with state: %d", state)
                return True
            else:
                rospy.logwarn("Navigation did not finish before timeout")
                return False
        else:
            # Fallback to direct topic publishing
            rospy.loginfo("Sending navigation goal to (%.2f, %.2f, %.2f) via topic", x, y, z)
            self.goal_pub.publish(goal_pose)
            rospy.loginfo("Goal published successfully")
            return True

def main():
    parser = argparse.ArgumentParser(description='Navigate to semantic objects')
    parser.add_argument('object', type=str, nargs='+', help='Object to navigate to (e.g., "chair" or "chair 2")')
    
    # Parse arguments
    args = parser.parse_args(rospy.myargv()[1:])
    object_query = ' '.join(args.object)
    
    navigator = ObjectNavigator()
    result = navigator.navigate_to_object(object_query)
    
    if result:
        rospy.loginfo("Navigation command completed successfully")
        return 0
    else:
        rospy.logerr("Navigation command failed")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass