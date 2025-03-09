#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import tf

class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node('pointcloud_to_map', anonymous=True)
        
        # Parameters
        self.resolution = rospy.get_param('~resolution', 0.05)  # Map resolution (meters/cell)
        self.height_min = rospy.get_param('~height_min', 0.1)   # Minimum height to consider for obstacles
        self.height_max = rospy.get_param('~height_max', 1.0)   # Maximum height to consider for obstacles
        self.grid_width = rospy.get_param('~grid_width', 20.0)  # Width of the grid in meters
        self.grid_height = rospy.get_param('~grid_height', 20.0)  # Height of the grid in meters
        
        # Create 2D grid
        self.width_cells = int(self.grid_width / self.resolution)
        self.height_cells = int(self.grid_height / self.resolution)
        self.grid = np.ones((self.height_cells, self.width_cells), dtype=np.int8) * -1  # Unknown
        
        # Publisher
        self.map_pub = rospy.Publisher('/orb_slam3/occupancy_grid', OccupancyGrid, queue_size=1)
        
        # Subscriber
        self.cloud_sub = rospy.Subscriber('/orb_slam3/map_points', PointCloud2, self.cloud_callback)
        
        rospy.spin()
    
    def cloud_callback(self, msg):
        # Reset grid
        self.grid = np.ones((self.height_cells, self.width_cells), dtype=np.int8) * -1  # Unknown
        
        # Extract points from point cloud
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            
            # Check if point is within our height constraints (for 2D navigation)
            if z < self.height_min or z > self.height_max:
                continue
            
            # Convert world coordinates to grid cell
            grid_x = int((x + self.grid_width/2) / self.resolution)
            grid_y = int((y + self.grid_height/2) / self.resolution)
            
            # Check if within grid bounds
            if 0 <= grid_x < self.width_cells and 0 <= grid_y < self.height_cells:
                # Mark as occupied
                self.grid[grid_y, grid_x] = 100  # Occupied
                
                # Mark neighboring cells as free (simple ray-tracing from origin)
                origin_x = int(self.width_cells / 2)
                origin_y = int(self.height_cells / 2)
                self.mark_free_cells(origin_x, origin_y, grid_x, grid_y)
        
        # Create and publish occupancy grid message
        self.publish_map(msg.header.stamp)
    
    def mark_free_cells(self, x0, y0, x1, y1):
        """Simple Bresenham line algorithm to mark cells between origin and obstacle as free"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.width_cells and 0 <= y0 < self.height_cells:
                # Only mark as free if not already marked as occupied
                if self.grid[y0, x0] != 100:
                    self.grid[y0, x0] = 0  # Free
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
    
    def publish_map(self, stamp):
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = "world"
        
        # Set metadata
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width_cells
        grid_msg.info.height = self.height_cells
        grid_msg.info.origin.position.x = -self.grid_width / 2
        grid_msg.info.origin.position.y = -self.grid_height / 2
        grid_msg.info.origin.position.z = 0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Flatten grid and convert to list
        grid_msg.data = self.grid.flatten().tolist()
        
        # Publish
        self.map_pub.publish(grid_msg)

if __name__ == '__main__':
    try:
        PointCloudToOccupancyGrid()
    except rospy.ROSInterruptException:
        pass