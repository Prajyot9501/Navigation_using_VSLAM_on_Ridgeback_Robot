#!/usr/bin/env python3

import rospy
import json
import argparse
import sys
from semantic_mapping.srv import QueryObject

class SemanticQueryClient:
    def __init__(self):
        rospy.init_node('semantic_query_client', anonymous=True)
        self.service_name = '/semantic_map/query'
        
        # Wait for service to be available
        rospy.loginfo(f"Waiting for service {self.service_name}...")
        rospy.wait_for_service(self.service_name)
        self.query_service = rospy.ServiceProxy(self.service_name, QueryObject)
        rospy.loginfo("Service available.")
    
    def query_object(self, query_str):
        """Query an object in the semantic map"""
        try:
            response = self.query_service(query=query_str)
            
            # Try to parse JSON response
            try:
                data = json.loads(response.response)
                rospy.loginfo("Object found:")
                rospy.loginfo(f"  Class: {data['class_name']}")
                rospy.loginfo(f"  Instance: {data['instance_id']}")
                rospy.loginfo(f"  Position: x={data['position']['x']:.2f}, y={data['position']['y']:.2f}, z={data['position']['z']:.2f}")
                rospy.loginfo(f"  Last seen: {data['time_since_last_seen']:.2f} seconds ago")
                
                return data
            except json.JSONDecodeError:
                # Not a JSON response, likely an error message
                rospy.loginfo(response.response)
                return None
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

def main():
    parser = argparse.ArgumentParser(description='Query the semantic map for objects')
    parser.add_argument('query', type=str, nargs='+', help='Object class or "class instance_id" to query')
    
    # Parse arguments
    args = parser.parse_args(rospy.myargv()[1:])
    query_str = ' '.join(args.query)
    
    client = SemanticQueryClient()
    result = client.query_object(query_str)
    
    if result:
        # Return success
        return 0
    else:
        # Return error code
        return 1

if __name__ == '__main__':
    sys.exit(main())