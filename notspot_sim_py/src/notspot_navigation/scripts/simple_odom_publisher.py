#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class SimpleOdomPublisher:
    def __init__(self):
        rospy.init_node('simple_odom_publisher')
        
        rospy.logwarn("INITIALIZING CORRECTED ODOMETRY PUBLISHER - EMERGENCY FIX VERSION")
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        # Position state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Previous IMU data for comparison
        self.prev_imu_orientation = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Robot height for correct positioning (matches default_height in RobotController.py)
        self.robot_height = 0.15
        
        # IMU data for orientation
        self.imu_orientation = None
        rospy.Subscriber('/notspot_imu/base_link_orientation', Imu, self.imu_callback)
        
        # Subscribe to cmd_vel to get velocity estimates
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        # TUNABLE PARAMETERS - Adjusted for emergency fix
        # Decay factor for velocities (when no commands are received)
        self.decay_factor = 0.9  # Less aggressive decay (was 0.85)
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # 500ms
        
        # Movement thresholds for stopping drift
        self.min_velocity_threshold = 0.0005  # Set low velocities to zero to reduce drift (was 0.002)
        
        # Enhanced covariance - tighter uncertainty (better position stability)
        self.position_covariance = 0.05  # Was 0.1
        self.orientation_covariance = 0.1  # Was 0.2
        
        # IMU orientation weight - how much to trust IMU vs. integration
        self.use_imu_yaw = True  # Use IMU for yaw orientation
        self.imu_yaw_weight = 0.5  # Weight between integrated yaw and IMU yaw (reduced from 0.8)
        
        # Flag for publishing transform
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
        # Debug variables
        self.drift_detect_counter = 0
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_orientation = 0.0
        self.drift_threshold = 0.001  # Minimum change to count as drift
        self.has_zero_velocity = True
        self.cmd_vel_updates = 0
        self.imu_updates = 0
        
        # Print configuration
        rospy.logwarn("=== EMERGENCY ODOMETRY CONFIGURATION ===")
        rospy.logwarn(f"Velocity decay factor: {self.decay_factor}")
        rospy.logwarn(f"Command timeout: {self.cmd_timeout.to_sec()} seconds")
        rospy.logwarn(f"Minimum velocity threshold: {self.min_velocity_threshold}")
        rospy.logwarn(f"Position covariance: {self.position_covariance}")
        rospy.logwarn(f"Orientation covariance: {self.orientation_covariance}")
        rospy.logwarn(f"Use IMU for yaw: {self.use_imu_yaw}")
        rospy.logwarn(f"IMU yaw weight: {self.imu_yaw_weight}")
        rospy.logwarn(f"Publishing TF: {self.publish_tf}")
        rospy.logwarn("==========================================")
        
        rospy.logwarn("Emergency Fix Odometry Publisher started")
        
        # Debug timer - Print status every 5 seconds
        rospy.Timer(rospy.Duration(5.0), self.debug_callback)
        
        # Main loop
        rate = rospy.Rate(20.0)  # 20Hz
        while not rospy.is_shutdown():
            self.publish_odom()
            self.check_tf_tree()
            rate.sleep()
    
    def debug_callback(self, event):
        """Print debug information periodically"""
        # Calculate position change since last debug output
        pos_change = math.sqrt((self.x - self.last_pos_x)**2 + (self.y - self.last_pos_y)**2)
        orientation_change = abs(self.th - self.last_orientation)
        
        # Check if there's significant drift while velocity should be zero
        if self.has_zero_velocity and (pos_change > self.drift_threshold or orientation_change > self.drift_threshold):
            self.drift_detect_counter += 1
            rospy.logwarn(f"DRIFT DETECTED! Position change: {pos_change:.6f}m, Orientation change: {orientation_change:.6f}rad")
        
        # Print current state
        rospy.loginfo("=== ODOMETRY STATE ===")
        rospy.loginfo(f"Position: x={self.x:.3f}, y={self.y:.3f}, θ={self.th:.3f}")
        rospy.loginfo(f"Velocity: vx={self.vx:.4f}, vy={self.vy:.4f}, vθ={self.vth:.4f}")
        
        if self.imu_orientation:
            rospy.loginfo(f"IMU Orientation: roll={self.roll:.3f}, pitch={self.pitch:.3f}, yaw={self.yaw:.3f}")
        else:
            rospy.loginfo("IMU Orientation: Not available")
        
        rospy.loginfo(f"Zero velocity state: {self.has_zero_velocity}")
        rospy.loginfo(f"Drift detections: {self.drift_detect_counter}")
        rospy.loginfo(f"Updates - cmd_vel: {self.cmd_vel_updates}, IMU: {self.imu_updates}")
        rospy.loginfo("=====================")
        
        # Store values for next comparison
        self.last_pos_x = self.x
        self.last_pos_y = self.y
        self.last_orientation = self.th
    
    def check_tf_tree(self):
        """Check and publish additional transforms if needed"""
        try:
            # Check if map->odom transform exists
            self.tf_listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(0.1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # If not, publish a simple identity transform as fallback
            if self.publish_tf:
                self.odom_broadcaster.sendTransform(
                    (0, 0, 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    "odom",
                    "map"
                )
                rospy.logdebug("Published map->odom transform (identity)")
    
    def imu_callback(self, msg):
        """Store orientation from IMU and extract roll, pitch, yaw"""
        self.imu_updates += 1
        self.imu_orientation = msg.orientation
        
        # Extract RPY
        orientation_q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_q)
        
        # Store previous orientation for comparison if not set
        if self.prev_imu_orientation is None:
            self.prev_imu_orientation = msg.orientation
            return
            
        # Compare with previous orientation to detect sensor problems
        prev_q = [self.prev_imu_orientation.x, self.prev_imu_orientation.y, 
                  self.prev_imu_orientation.z, self.prev_imu_orientation.w]
        prev_roll, prev_pitch, prev_yaw = euler_from_quaternion(prev_q)
        
        # Calculate angle differences
        roll_diff = abs(self.roll - prev_roll)
        pitch_diff = abs(self.pitch - prev_pitch)
        yaw_diff = abs(self.yaw - prev_yaw)
        
        # Check for large sudden changes (potential sensor issues)
        if roll_diff > 0.2 or pitch_diff > 0.2 or yaw_diff > 0.3:
            rospy.logwarn(f"Large IMU orientation jump detected! Roll: {roll_diff:.3f}, Pitch: {pitch_diff:.3f}, Yaw: {yaw_diff:.3f}")
            
        # Store current orientation as previous for next comparison
        self.prev_imu_orientation = msg.orientation
    
    def cmd_vel_callback(self, msg):
        """Update velocity estimates from cmd_vel"""
        self.cmd_vel_updates += 1
        
        # Previous velocity state for comparison
        prev_vx = self.vx
        prev_vy = self.vy
        prev_vth = self.vth
        
        # Update velocity state
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
        
        # Check if the velocities are effectively zero
        self.has_zero_velocity = (abs(self.vx) < self.min_velocity_threshold and 
                                  abs(self.vy) < self.min_velocity_threshold and 
                                  abs(self.vth) < self.min_velocity_threshold)
        
        # If velocities are below threshold, explicitly set to zero to avoid drift
        if self.has_zero_velocity:
            self.vx = 0.0
            self.vy = 0.0
            self.vth = 0.0
            if not (abs(prev_vx) < self.min_velocity_threshold and 
                    abs(prev_vy) < self.min_velocity_threshold and 
                    abs(prev_vth) < self.min_velocity_threshold):
                rospy.loginfo("🛑 Setting velocities to zero (below threshold)")
        
        # Update the last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Log velocity changes (only for non-zero changes)
        if not (abs(prev_vx - self.vx) < 0.001 and 
                abs(prev_vy - self.vy) < 0.001 and 
                abs(prev_vth - self.vth) < 0.001):
            rospy.loginfo(f"Velocity update: vx={self.vx:.4f}, vy={self.vy:.4f}, vth={self.vth:.4f}")
    
    def publish_odom(self):
        self.current_time = rospy.Time.now()
        
        # Check if we need to decay velocities (no cmd_vel received recently)
        if (self.current_time - self.last_cmd_time) > self.cmd_timeout:
            if not (self.vx == 0 and self.vy == 0 and self.vth == 0):
                rospy.logdebug(f"Decaying velocities: {self.vx:.4f}, {self.vy:.4f}, {self.vth:.4f} * {self.decay_factor}")
            
            self.vx *= self.decay_factor
            self.vy *= self.decay_factor
            self.vth *= self.decay_factor
            
            # If velocities are very small, set them to zero to prevent drift
            if abs(self.vx) < self.min_velocity_threshold:
                self.vx = 0.0
            if abs(self.vy) < self.min_velocity_threshold:
                self.vy = 0.0
            if abs(self.vth) < self.min_velocity_threshold:
                self.vth = 0.0
        
        # Compute dt
        dt = (self.current_time - self.last_time).to_sec()
        
        # CORRECTED: Transform velocities from robot's local frame to global frame
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        # Log velocity transformation for debugging
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001 or abs(delta_th) > 0.001:
            rospy.logwarn(f"Velocity transform: vx={self.vx:.4f}, vy={self.vy:.4f} -> dx={delta_x:.4f}, dy={delta_y:.4f}")
        
        # Only update position if there's actual movement
        if abs(delta_x) > 0 or abs(delta_y) > 0 or abs(delta_th) > 0:
            # Update position estimate
            self.x += delta_x
            self.y += delta_y
            
            # Update theta/yaw based on integration and/or IMU
            if self.use_imu_yaw and self.imu_orientation:
                # Blend integrated yaw with IMU yaw for more stability
                integrated_th = self.th + delta_th
                # Ensure yaw values are comparable (handle wraparound)
                while self.yaw - integrated_th > math.pi:
                    integrated_th += 2 * math.pi
                while self.yaw - integrated_th < -math.pi:
                    integrated_th -= 2 * math.pi
                    
                # Weighted blend
                self.th = (1 - self.imu_yaw_weight) * integrated_th + self.imu_yaw_weight * self.yaw
                
                # Debug output for significant orientation changes
                if abs(delta_th) > 0.01:
                    rospy.loginfo(f"Orientation blend: integrated={integrated_th:.4f}, imu={self.yaw:.4f}, result={self.th:.4f}")
            else:
                # Standard integration if IMU not used or not available
                self.th += delta_th
                
            # Log significant movements
            if abs(delta_x) > 0.005 or abs(delta_y) > 0.005 or abs(delta_th) > 0.01:
                rospy.loginfo(f"Position update: dx={delta_x:.4f}, dy={delta_y:.4f}, dth={delta_th:.4f}")
                rospy.loginfo(f"New position: x={self.x:.4f}, y={self.y:.4f}, th={self.th:.4f}")
        
        # Create quaternion from yaw or use IMU orientation if available
        if self.imu_orientation:
            # Option 1: Use IMU orientation directly (roll and pitch from IMU, yaw from integrated or blended)
            if self.use_imu_yaw:
                odom_quat = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.th)
            else:
                # Option 2: Use full IMU orientation
                odom_quat = [
                    self.imu_orientation.x,
                    self.imu_orientation.y, 
                    self.imu_orientation.z,
                    self.imu_orientation.w
                ]
        else:
            # Fallback to integrated yaw only (no roll/pitch)
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Publish transform over tf
        if self.publish_tf:
            # Create a TransformStamped message for better structure
            transform = TransformStamped()
            transform.header.stamp = self.current_time
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = self.robot_height
            transform.transform.rotation.x = odom_quat[0]
            transform.transform.rotation.y = odom_quat[1]
            transform.transform.rotation.z = odom_quat[2]
            transform.transform.rotation.w = odom_quat[3]
            
            # Send the transform directly using sendTransformMessage
            self.odom_broadcaster.sendTransformMessage(transform)
            
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
        odom.pose.covariance[0] = self.position_covariance   # x
        odom.pose.covariance[7] = self.position_covariance   # y
        odom.pose.covariance[14] = self.position_covariance  # z
        odom.pose.covariance[21] = self.orientation_covariance  # roll
        odom.pose.covariance[28] = self.orientation_covariance  # pitch
        odom.pose.covariance[35] = self.orientation_covariance  # yaw
        
        odom.twist.covariance = odom.pose.covariance
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        self.last_time = self.current_time

if __name__ == '__main__':
    try:
        SimpleOdomPublisher()
    except rospy.ROSInterruptException:
        pass