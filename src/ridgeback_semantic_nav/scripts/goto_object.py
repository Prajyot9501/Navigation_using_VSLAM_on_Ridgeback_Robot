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
        self.approach_distance = rospy.get_param('~approach_distance', 0.4)  # meters
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
                
                # Clear costmaps before navigation
                try:
                    rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
                    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                    clear_costmaps()
                    rospy.loginfo("Cleared costmaps before navigation")
                except (rospy.ROSException, rospy.ServiceException) as e:
                    rospy.logwarn("Could not clear costmaps: %s", str(e))
                
                # Publish a marker at the target location
                self.publish_target_marker(x, y, z, object_data['class_name'])
                
                # Send the navigation goal via topic (more reliable for tight spaces)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = rospy.Time.now()
                
                # Set the position
                goal_pose.pose.position.x = x
                goal_pose.pose.position.y = y
                goal_pose.pose.position.z = 0.0  # Keep the robot on the ground
                
                # Set orientation (identity quaternion)
                goal_pose.pose.orientation.w = 1.0
                
                rospy.loginfo("Sending navigation goal to position: (%.2f, %.2f, 0.0)", x, y)
                
                # Publish goal via topic
                if hasattr(self, 'goal_pub'):
                    self.goal_pub.publish(goal_pose)
                    rospy.loginfo("Goal published via topic")
                    return True
                else:
                    # Use action client as backup
                    goal = MoveBaseGoal()
                    goal.target_pose = goal_pose
                    self.move_base_client.send_goal(goal)
                    rospy.loginfo("Goal sent via action client")
                    return True
                
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
        
        # Set scale and color (bright green)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
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
        from std_srvs.srv import Empty
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass