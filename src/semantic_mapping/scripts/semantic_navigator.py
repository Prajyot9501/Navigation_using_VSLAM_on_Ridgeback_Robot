#!/usr/bin/env python3

import rospy
import json
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from semantic_mapping.srv import QueryObject

class SemanticNavigator:
    def __init__(self):
        rospy.init_node('semantic_navigator', anonymous=True)
        
        # Parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.min_confidence = rospy.get_param('~min_confidence', 0.7)
        
        # Wait for semantic map service
        rospy.loginfo("Waiting for semantic map query service...")
        rospy.wait_for_service('/semantic_map/query')
        self.query_service = rospy.ServiceProxy('/semantic_map/query', QueryObject)
        
        # Set up move_base client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        
        # Set up simple interface service
        self.navigate_service = rospy.Service('/semantic_navigation/navigate_to', QueryObject, self.handle_navigate_to)
        
        # Set up direct goal subscribers
        self.goal_sub = rospy.Subscriber('/semantic_navigation/goal', String, self.handle_goal)
        
        rospy.loginfo("Semantic navigator ready!")
    
    def handle_goal(self, msg):
        """Handle direct goal specification via topic"""
        self.navigate_to_object(msg.data)
    
    def handle_navigate_to(self, req):
        """Handle navigation service request"""
        result = self.navigate_to_object(req.query)
        return result
    
    def navigate_to_object(self, query_str):
        """Query the semantic map and navigate to the object"""
        try:
            # Query the semantic map
            rospy.loginfo(f"Querying semantic map for: {query_str}")
            response = self.query_service(query_str)
            
            try:
                # Parse the response
                data = json.loads(response.response)
                
                # Check if the object exists
                if 'class_name' not in data:
                    rospy.logwarn(f"Object not found: {query_str}")
                    return "Object not found in semantic map."
                
                rospy.loginfo(f"Found {data['class_name']} (instance {data['instance_id']}) at position: " +
                              f"x={data['position']['x']:.2f}, y={data['position']['y']:.2f}, z={data['position']['z']:.2f}")
                
                # Set navigation goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.world_frame
                goal.target_pose.header.stamp = rospy.Time.now()
                
                # Set position
                goal.target_pose.pose.position.x = data['position']['x']
                goal.target_pose.pose.position.y = data['position']['y']
                goal.target_pose.pose.position.z = 0.0  # Keep on ground plane
                
                # Set orientation (just face toward the object from 1m away)
                # For simplicity, we're using identity quaternion (no rotation)
                goal.target_pose.pose.orientation.w = 1.0
                
                # Send the goal and wait for result
                rospy.loginfo(f"Navigating to {data['class_name']}...")
                self.move_base_client.send_goal(goal)
                
                # Wait for result with timeout
                success = self.move_base_client.wait_for_result(rospy.Duration(60.0))
                
                if success:
                    result = self.move_base_client.get_result()
                    if result:
                        rospy.loginfo(f"Navigation to {data['class_name']} successful!")
                        return f"Successfully navigated to {data['class_name']} (instance {data['instance_id']})."
                    else:
                        rospy.logwarn(f"Navigation to {data['class_name']} failed.")
                        return f"Failed to navigate to {data['class_name']}."
                else:
                    rospy.logwarn("Navigation timed out.")
                    self.move_base_client.cancel_goal()
                    return "Navigation timed out."
                
            except json.JSONDecodeError:
                # Not a JSON response, likely an error message
                rospy.logwarn(f"Error response from semantic map: {response.response}")
                return response.response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return f"Service call failed: {e}"

if __name__ == '__main__':
    try:
        navigator = SemanticNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass