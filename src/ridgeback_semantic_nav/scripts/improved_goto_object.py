#!/usr/bin/env python3

import rospy
import json
import sys
import argparse
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
from semantic_mapping.srv import QueryObject
from visualization_msgs.msg import Marker

class ImprovedObjectNavigator:
    def __init__(self):
        rospy.init_node('improved_object_navigator', anonymous=True)
        
        # Set up TF2 for coordinate frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Goal publisher - direct topic approach is most reliable
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # Visualization marker
        self.marker_pub = rospy.Publisher('/navigation_target', Marker, queue_size=1)
        
        # Wait for publisher connections
        rospy.sleep(1.0)
        
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
                
                rospy.loginfo("Found %s at position: (%.2f, %.2f, %.2f) in world frame", 
                             object_data['class_name'], x, y, z)
                
                # Publish a marker at the target location
                self.publish_target_marker(x, y, z, object_data['class_name'])
                
                # Get the current robot position to determine approach vector
                try:
                    # Create a navigation goal
                    goal_msg = PoseStamped()
                    
                    # THIS IS CRITICAL: Use map frame for navigation goal
                    goal_msg.header.frame_id = "map"
                    goal_msg.header.stamp = rospy.Time.now()
                    
                    # Set the position - transform from world to map if needed
                    world_point = PoseStamped()
                    world_point.header.frame_id = "world"
                    world_point.header.stamp = rospy.Time.now()
                    world_point.pose.position.x = x
                    world_point.pose.position.y = y
                    world_point.pose.position.z = 0.0
                    world_point.pose.orientation.w = 1.0
                    
                    try:
                        # Transform from world to map frame
                        map_point = self.tf_buffer.transform(world_point, "map", rospy.Duration(1.0))
                        goal_msg = map_point
                        
                        rospy.loginfo("Transformed goal: world (%.2f, %.2f) -> map (%.2f, %.2f)",
                                    x, y, map_point.pose.position.x, map_point.pose.position.y)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        rospy.logwarn("TF transform failed: %s. Using direct world coordinates as map coordinates.", e)
                        # Fallback: Use world coordinates directly
                        goal_msg.pose.position.x = x
                        goal_msg.pose.position.y = y
                        goal_msg.pose.position.z = 0.0
                        goal_msg.pose.orientation.w = 1.0
                    
                    # Send the goal
                    rospy.loginfo("Publishing navigation goal to (%.2f, %.2f) in frame %s",
                                goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.header.frame_id)
                    self.goal_pub.publish(goal_msg)
                    
                    rospy.loginfo("Navigation goal published!")
                    return True
                    
                except Exception as e:
                    rospy.logerr("Navigation error: %s", str(e))
                    return False
                
            except json.JSONDecodeError:
                rospy.logerr("Failed to parse response as JSON: %s", response.response)
                return False
                
        except rospy.ROSException as e:
            rospy.logerr("Service error: %s", str(e))
            return False
    
    def publish_target_marker(self, x, y, z, object_name):
        """Publish a visualization marker at the target location"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navigation_targets"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        # Set scale and color (bright green)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Set text
        marker.text = f"Target: {object_name}"
        
        # Set lifetime
        marker.lifetime = rospy.Duration(0)  # 0 means forever
        
        # Publish the marker
        self.marker_pub.publish(marker)
        
def main():
    parser = argparse.ArgumentParser(description='Navigate to semantic objects')
    parser.add_argument('object', type=str, nargs='+', help='Object to navigate to (e.g., "chair" or "chair 2")')
    
    # Parse arguments
    args = parser.parse_args(rospy.myargv()[1:])
    object_query = ' '.join(args.object)
    
    navigator = ImprovedObjectNavigator()
    result = navigator.navigate_to_object(object_query)
    
    if result:
        rospy.loginfo("Navigation command completed")
        return 0
    else:
        rospy.logerr("Navigation command failed")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass