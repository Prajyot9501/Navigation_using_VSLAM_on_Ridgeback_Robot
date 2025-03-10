#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import struct

class TestPointcloudPublisher:
    def __init__(self):
        rospy.init_node('test_pointcloud_publisher', anonymous=True)
        self.pointcloud_pub = rospy.Publisher('/orb_slam3/map_points', PointCloud2, queue_size=1)
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.loginfo("Test pointcloud publisher initialized")
        
    def publish_test_pointcloud(self):
        # Create a list to store points
        points = []
        
        # Generate a grid of points (-5 to 5 in x and y, z=0)
        for x in np.arange(-5.0, 5.0, 0.2):
            for y in np.arange(-5.0, 5.0, 0.2):
                # Rainbow coloring
                r = int(255 * (x + 5.0) / 10.0)
                g = int(255 * (y + 5.0) / 10.0)
                b = 255
                
                # Create RGB value
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                
                # Add point with color
                points.append([x, y, 0.0, rgb])
        
        # Create header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        
        # Define fields for XYZRGB pointcloud
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        
        # Create PointCloud2 message
        point_cloud = pc2.create_cloud(header, fields, points)
        
        # Publish
        rospy.loginfo(f"Publishing test pointcloud with {len(points)} points")
        self.pointcloud_pub.publish(point_cloud)
    
    def run(self):
        while not rospy.is_shutdown():
            self.publish_test_pointcloud()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestPointcloudPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass