#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu

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
        
        # Robot height for correct positioning
        self.robot_height = 0.15
        
        # IMU data for orientation
        self.imu_orientation = None
        rospy.Subscriber('/notspot_imu/base_link_orientation', Imu, self.imu_callback)
        
        # Subscribe to cmd_vel to get velocity estimates
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        # Threshold for considering velocities as zero
        self.velocity_threshold = 0.0001
        
        # Decay factor for velocities (when no commands are received)
        self.decay_factor = 0.95
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # 500ms
        
        # Flag for publishing transform
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
        rospy.loginfo("Simple Odometry Publisher started")
        
        # Main loop
        rate = rospy.Rate(20.0)  # 20Hz
        while not rospy.is_shutdown():
            self.publish_odom()
            rate.sleep()
    
    def imu_callback(self, msg):
        """Store orientation from IMU"""
        self.imu_orientation = msg.orientation
        
        # Extract yaw from IMU quaternion
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        # Update our heading based on IMU
        # Optional: use a complementary filter here if you don't want to fully 
        # trust the IMU (e.g., self.th = 0.95 * self.th + 0.05 * yaw)
        self.th = yaw
    
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
            # Apply thresholding to small velocities to combat drift
            if abs(self.vx) < self.velocity_threshold:
                self.vx = 0.0
            if abs(self.vy) < self.velocity_threshold:
                self.vy = 0.0
            if abs(self.vth) < self.velocity_threshold:
                self.vth = 0.0
            
            # Apply decay for remaining non-zero values
            self.vx *= self.decay_factor
            self.vy *= self.decay_factor
            self.vth *= self.decay_factor
        
        # Compute dt
        dt = (self.current_time - self.last_time).to_sec()
        
        # Use the same approach as the C++ code for computing position changes
        # This accounts for the robot's current orientation when calculating position change
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        # Update position estimate
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Normalize angle to stay within -π to π
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        
        # Create quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Publish transform over tf
        if self.publish_tf:
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, self.robot_height),
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
        odom.pose.pose = Pose(Point(self.x, self.y, self.robot_height), Quaternion(*odom_quat))
        
        # Set the velocity
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        
        # Add covariance (lower values = higher certainty)
        odom.pose.covariance[0] = 0.0   # x
        odom.pose.covariance[7] = 0.0   # y
        odom.pose.covariance[14] = 0.0  # z
        odom.pose.covariance[21] = 0.0  # roll
        odom.pose.covariance[28] = 0.0  # pitch
        odom.pose.covariance[35] = 0.0  # yaw
        
        odom.twist.covariance = odom.pose.covariance
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        self.last_time = self.current_time
        
        # Debug output - comment out when not needed
        if (self.current_time.to_sec() % 5) < 0.1:  # print every ~5 seconds
            rospy.loginfo(f"Odom: x={self.x:.3f}, y={self.y:.3f}, θ={self.th:.3f}, " +
                         f"vx={self.vx:.3f}, vy={self.vy:.3f}, vθ={self.vth:.3f}")

if __name__ == '__main__':
    try:
        SimpleOdomPublisher()
    except rospy.ROSInterruptException:
        pass