#!/usr/bin/env python3

import rospy
import tf2_ros
import tf.transformations
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FakeVelocityController:
    def __init__(self):
        rospy.init_node('fake_velocity_controller', anonymous=True)
        
        # Robot's current position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Create an Odometry publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Timer to update TF and odometry
        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.update_tf)  # 20Hz
        
        rospy.loginfo("Fake velocity controller initialized")
    
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate time since last callback
        now = rospy.Time.now()
        if not hasattr(self, 'last_time'):
            self.last_time = now
            return
        
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        
        # Simple motion model - update position based on velocities
        if abs(angular_z) > 0.001:
            # Circular motion
            radius = linear_x / angular_z
            delta_theta = angular_z * dt
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
            self.theta += delta_theta
        else:
            # Straight line motion
            self.x += linear_x * dt * math.cos(self.theta)
            self.y += linear_x * dt * math.sin(self.theta)
        
        # Normalize theta
        self.theta = self.normalize_angle(self.theta)
        
        rospy.loginfo(f"Robot position updated: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
    
    def update_tf(self, event):
        # Create transform from odom to base_link
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Set rotation (convert theta to quaternion)
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Publish odometry
        self.odom_pub.publish(odom)
    
    def normalize_angle(self, angle):
        # Normalize angle to [-π, π]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    try:
        controller = FakeVelocityController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass