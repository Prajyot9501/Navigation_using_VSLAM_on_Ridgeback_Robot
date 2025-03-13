#!/usr/bin/env python3

import rospy
import json
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from semantic_mapping.srv import QueryObject, QueryObjectResponse

class EnhancedSemanticNavigator:
    def __init__(self):
        rospy.init_node('enhanced_semantic_navigator', anonymous=False)
        
        # Parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.min_confidence = rospy.get_param('~min_confidence', 0.7)
        self.approach_distance = rospy.get_param('~approach_distance', 0.75)  # meters
        
        rospy.loginfo("Starting enhanced semantic navigator...")
        rospy.loginfo("Configured with: world_frame=%s, min_confidence=%f, approach_distance=%f", 
                     self.world_frame, self.min_confidence, self.approach_distance)
        
        # Wait for semantic map service
        semantic_service_name = '/semantic_map/query'
        rospy.loginfo("Waiting for semantic map query service at %s...", semantic_service_name)
        
        try:
            rospy.wait_for_service(semantic_service_name, timeout=10.0)
            self.query_service = rospy.ServiceProxy(semantic_service_name, QueryObject)
            rospy.loginfo("Connected to semantic map service")
        except rospy.ROSException as e:
            rospy.logerr("Semantic map service not available: %s", str(e))
            rospy.logerr("Make sure the semantic_mapper node is running")
            self.query_service = None
            
        # Set up move_base client
        move_base_name = 'move_base'
        rospy.loginfo("Connecting to move_base action server...")
        
        try:
            self.move_base_client = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
            server_available = self.move_base_client.wait_for_server(timeout=rospy.Duration(10.0))
            
            if server_available:
                rospy.loginfo("Connected to move_base action server")
            else:
                rospy.logerr("Timed out waiting for move_base action server")
                self.move_base_client = None
        except Exception as e:
            rospy.logerr("Error connecting to move_base: %s", str(e))
            self.move_base_client = None
            
        # Set up navigation service and topic
        self.navigate_service = rospy.Service('/semantic_navigation/navigate_to', 
                                             QueryObject, self.handle_navigate_to)
        rospy.loginfo("Navigation service registered at /semantic_navigation/navigate_to")
        
        # Set up goal subscriber
        self.goal_sub = rospy.Subscriber('/semantic_navigation/goal', String, self.handle_goal)
        rospy.loginfo("Navigation goal topic subscriber registered at /semantic_navigation/goal")
        
        rospy.loginfo("Enhanced semantic navigator is ready!")
    
    def handle_goal(self, msg):
        """Handle direct goal specification via topic"""
        rospy.loginfo("Received navigation request via topic: %s", msg.data)
        self.navigate_to_object(msg.data)
    
    def handle_navigate_to(self, req):
        """Handle navigation service request"""
        rospy.loginfo("Received navigation request via service: %s", req.query)
        result = self.navigate_to_object(req.query)
        return QueryObjectResponse(response=result)
    
    def navigate_to_object(self, query_str):
        """Query the semantic map and navigate to the object"""
        if not self.query_service:
            error_msg = "Cannot navigate: Semantic map service not available"
            rospy.logerr(error_msg)
            return error_msg
            
        if not self.move_base_client:
            error_msg = "Cannot navigate: move_base action client not available"
            rospy.logerr(error_msg)
            return error_msg
        
        try:
            # Query the semantic map
            rospy.loginfo("Querying semantic map for: %s", query_str)
            response = self.query_service(query=query_str)
            rospy.loginfo("Semantic map response: %s", response.response)
            
            try:
                # Parse the response
                data = json.loads(response.response)
                
                # Check if the object exists
                if 'class_name' not in data:
                    error_msg = f"Object not found: {query_str}"
                    rospy.logwarn(error_msg)
                    return error_msg
                
                rospy.loginfo("Found %s (instance %d) at position: x=%.2f, y=%.2f, z=%.2f",
                             data['class_name'], data['instance_id'],
                             data['position']['x'], data['position']['y'], data['position']['z'])
                
                # Set navigation goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.world_frame
                goal.target_pose.header.stamp = rospy.Time.now()
                
                # Set position (with approach distance)
                obj_x = data['position']['x']
                obj_y = data['position']['y']
                
                # For now, just navigate directly to the object position
                # In a more advanced implementation, we could compute an approach vector
                goal.target_pose.pose.position.x = obj_x
                goal.target_pose.pose.position.y = obj_y
                goal.target_pose.pose.position.z = 0.0  # Keep on ground plane
                
                # Set orientation (just face toward the object)
                # For simplicity, we're using identity quaternion (no rotation)
                goal.target_pose.pose.orientation.w = 1.0
                
                # Send the goal
                rospy.loginfo("Navigating to %s...", data['class_name'])
                self.move_base_client.send_goal(goal)
                
                # Wait for result with timeout
                rospy.loginfo("Waiting for navigation to complete...")
                success = self.move_base_client.wait_for_result(rospy.Duration(60.0))
                
                if success:
                    result = self.move_base_client.get_result()
                    if result:
                        success_msg = f"Successfully navigated to {data['class_name']} (instance {data['instance_id']})."
                        rospy.loginfo(success_msg)
                        return success_msg
                    else:
                        error_msg = f"Failed to navigate to {data['class_name']}."
                        rospy.logwarn(error_msg)
                        return error_msg
                else:
                    error_msg = "Navigation timed out."
                    rospy.logwarn(error_msg)
                    self.move_base_client.cancel_goal()
                    return error_msg
                
            except json.JSONDecodeError as e:
                # Not a JSON response, likely an error message
                error_msg = f"Error parsing semantic map response: {str(e)}"
                rospy.logwarn(error_msg)
                return response.response
            
        except rospy.ServiceException as e:
            error_msg = f"Service call failed: {str(e)}"
            rospy.logerr(error_msg)
            return error_msg

if __name__ == '__main__':
    try:
        navigator = EnhancedSemanticNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass