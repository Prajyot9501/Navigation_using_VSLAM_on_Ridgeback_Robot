#!/usr/bin/env python3

import rospy
import tf2_ros
import json
import sqlite3
import os
from semantic_mapping.srv import QueryObject

def check_tf_frames():
    rospy.init_node('check_tf_objects', anonymous=True)
    
    # Connect to the semantic database
    db_path = os.path.expanduser('~/semantic_map.db')
    if not os.path.exists(db_path):
        rospy.logerr(f"Database not found at {db_path}")
        return
        
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    
    # Get object positions from database
    c.execute("SELECT class_name, instance_id, x, y, z FROM objects")
    objects = c.fetchall()
    
    rospy.loginfo("Objects in database:")
    for obj in objects:
        class_name, instance_id, x, y, z = obj
        rospy.loginfo(f"  {class_name} {instance_id}: position in world = ({x:.2f}, {y:.2f}, {z:.2f})")
    
    conn.close()
    
    # Check TF frames
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Wait for TF to be available
    rospy.sleep(1.0)
    
    # Check for key TF frames
    frames_to_check = [
        ('world', 'map'),
        ('map', 'odom'),
        ('odom', 'base_link')
    ]
    
    rospy.loginfo("Checking TF frames:")
    for parent, child in frames_to_check:
        try:
            trans = tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo(f"  Transform {parent} -> {child}: exists")
            rospy.loginfo(f"    Translation: ({trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f}, {trans.transform.translation.z:.2f})")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"  Transform {parent} -> {child}: not available - {e}")
    
    # Try to check the map dimensions from parameter server
    try:
        if rospy.has_param('/map_server/width') and rospy.has_param('/map_server/height'):
            width = rospy.get_param('/map_server/width')
            height = rospy.get_param('/map_server/height')
            resolution = rospy.get_param('/map_server/resolution')
            rospy.loginfo(f"Map dimensions: {width} x {height} cells @ {resolution} m/cell")
            rospy.loginfo(f"Map size: {width * resolution:.2f} x {height * resolution:.2f} meters")
        else:
            rospy.logwarn("Map dimensions not available in parameter server")
    except Exception as e:
        rospy.logwarn(f"Error getting map dimensions: {e}")
    
    # Test querying an object
    try:
        rospy.wait_for_service('/semantic_map/query', timeout=2.0)
        query_service = rospy.ServiceProxy('/semantic_map/query', QueryObject)
        
        # Try to query for a chair
        response = query_service(query="chair")
        rospy.loginfo(f"Query for 'chair' response: {response.response}")
        
        try:
            obj_data = json.loads(response.response)
            x, y, z = obj_data['position']['x'], obj_data['position']['y'], obj_data['position']['z']
            rospy.loginfo(f"Chair position in world frame: ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Try to check if this point is within map bounds
            try:
                # This is approximate since we don't know the exact map origin
                if -1.6 <= x <= 1.6 and -1.55 <= y <= 1.55:  # Half map size
                    rospy.loginfo("Position appears to be within reasonable map bounds")
                else:
                    rospy.logwarn("Position may be outside reasonable map bounds")
            except:
                pass
        except:
            pass
    except Exception as e:
        rospy.logwarn(f"Error querying object: {e}")

if __name__ == '__main__':
    try:
        check_tf_frames()
    except rospy.ROSInterruptException:
        pass