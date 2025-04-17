#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu

class SimpleOdomPublisher:
    def __init__(self):
        rospy.init_node('simple_odom_publisher')
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Robot height for correct positioning (matches default_height in RobotController.py)
        self.robot_height = 0.15
        
        # IMU data for orientation
        self.imu_orientation = None
        rospy.Subscriber('/notspot_imu/base_link_orientation', Imu, self.imu_callback)
        
        # Subscribe to cmd_vel to get velocity estimates
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        # Decay factor for velocities (when no commands are received)
        self.decay_factor = 0.95
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # 500ms
        
        # Flag for publishing transform
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
        rospy.loginfo("Simple Odometry Publisher started")
        
        # Wait for a moment to make sure tf is ready
        rospy.sleep(1.0)
        
        # Main loop
        rate = rospy.Rate(20.0)  # 20Hz
        while not rospy.is_shutdown():
            self.publish_odom()
            rate.sleep()
    
    def imu_callback(self, msg):
        """Store orientation from IMU"""
        self.imu_orientation = msg.orientation
    
    def cmd_vel_callback(self, msg):
        """Update velocity estimates from cmd_vel"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
        self.last_cmd_time = rospy.Time.now()
    
    def publish_odom(self):
        self.current_time = rospy.Time.now()
        
        # Check if we need to decay velocities (no cmd_vel received recently)
        if (self.current_time - self.last_cmd_time) > self.cmd_timeout:
            self.vx *= self.decay_factor
            self.vy *= self.decay_factor
            self.vth *= self.decay_factor
        
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
        
        # Create quaternion from yaw or use IMU orientation if available
        if self.imu_orientation:
            odom_quat = [
                self.imu_orientation.x,
                self.imu_orientation.y,
                self.imu_orientation.z,
                self.imu_orientation.w
            ]
        else:
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Publish transform over tf
        if self.publish_tf:
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, self.robot_height),  # Include robot height for proper visualization
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )
            
            # Also broadcast base_link to base_footprint
            self.odom_broadcaster.sendTransform(
                (0, 0, 0),  # The footprint is directly below the base_link
                tf.transformations.quaternion_from_euler(0, 0, 0),
                self.current_time,
                "base_footprint",
                "base_link"
            )
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, self.robot_height), Quaternion(*odom_quat))
        
        # Set the velocity
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        
        # Add covariance (needed by some packages)
        # Lower values = higher certainty
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[14] = 0.1  # z
        odom.pose.covariance[21] = 0.2  # roll
        odom.pose.covariance[28] = 0.2  # pitch
        odom.pose.covariance[35] = 0.2  # yaw
        
        odom.twist.covariance = odom.pose.covariance
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        self.last_time = self.current_time

if __name__ == '__main__':
    try:
        SimpleOdomPublisher()
    except rospy.ROSInterruptException:
        pass