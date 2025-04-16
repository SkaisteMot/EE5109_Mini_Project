#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class SimpleOdomPublisher:
    def __init__(self):
        rospy.init_node('simple_odom_publisher')
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Subscribe to cmd_vel to get velocity estimates
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        self.rate = rospy.Rate(20.0)  # 20Hz
        
        # Main loop
        while not rospy.is_shutdown():
            self.publish_odom()
            self.rate.sleep()
    
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
    
    def publish_odom(self):
        self.current_time = rospy.Time.now()
        
        # Compute dt
        dt = (self.current_time - self.last_time).to_sec()
        
        # Compute distance traveled
        delta_x = (self.vx * dt) 
        delta_y = (self.vy * dt)
        delta_th = (self.vth * dt)
        
        # Update position estimate
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Create quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Publish transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        
        # Set the velocity
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        self.last_time = self.current_time

if __name__ == '__main__':
    try:
        SimpleOdomPublisher()
    except rospy.ROSInterruptException:
        pass