#!/usr/bin/env python3

import rospy
import sys
import argparse
from std_msgs.msg import String
from semantic_mapping.srv import QueryObject

class ObjectNavigator:
    def __init__(self):
        rospy.init_node('object_navigator_client', anonymous=True)
        
        # Create publisher to send goals
        self.goal_pub = rospy.Publisher('/semantic_navigation/goal', String, queue_size=1)
        
        # Wait a bit for the publisher to connect
        rospy.sleep(0.5)
        rospy.loginfo("Object navigator client initialized")
        
    def navigate_to(self, object_query):
        """Send a navigation goal to the specified object"""
        rospy.loginfo(f"Sending navigation request: {object_query}")
        
        # Try service method first
        try:
            service_name = '/semantic_navigation/navigate_to'
            rospy.loginfo(f"Trying service method at {service_name}...")
            
            # Wait for service with timeout
            service_available = rospy.wait_for_service(service_name, timeout=2.0)
            
            # Call the service
            query_service = rospy.ServiceProxy(service_name, QueryObject)
            response = query_service(query=object_query)
            rospy.loginfo(f"Navigation service response: {response.response}")
            return True
            
        except rospy.ROSException as e:
            rospy.logwarn(f"Navigation service not available: {e}")
            rospy.loginfo("Falling back to topic method...")
            
            # Fallback to topic method
            self.goal_pub.publish(String(object_query))
            rospy.loginfo(f"Published navigation goal to topic: {object_query}")
            return True
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
    def query_object(self, object_query):
        """Query object information without navigating"""
        rospy.loginfo(f"Querying object: {object_query}")
        
        # Wait for service
        service_name = '/semantic_map/query'
        try:
            rospy.loginfo(f"Waiting for service {service_name}...")
            
            # Wait for service with timeout
            service_available = rospy.wait_for_service(service_name, timeout=5.0)
            
            # Call the service
            query_service = rospy.ServiceProxy(service_name, QueryObject)
            response = query_service(query=object_query)
            rospy.loginfo(f"Query response: {response.response}")
            return True
            
        except rospy.ROSException as e:
            rospy.logerr(f"Query service not available: {e}")
            rospy.logerr("Make sure the semantic_mapper node is running")
            return False
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

def main():
    parser = argparse.ArgumentParser(description='Navigate to semantic objects')
    parser.add_argument('query', type=str, nargs='+', help='Object to navigate to (e.g., "chair" or "chair 2")')
    parser.add_argument('--query-only', action='store_true', help='Only query the object without navigating')
    
    # Parse arguments
    args = parser.parse_args(rospy.myargv()[1:])
    object_query = ' '.join(args.query)
    
    navigator = ObjectNavigator()
    
    if args.query_only:
        result = navigator.query_object(object_query)
    else:
        result = navigator.navigate_to(object_query)
    
    if result:
        return 0
    else:
        return 1

if __name__ == '__main__':
    sys.exit(main())