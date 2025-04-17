#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoints:
    def __init__(self):
        rospy.init_node('cmd_vel_to_joints')
        
        # Parameters matching trot gait speeds
        self.max_x_velocity = 0.024  # [m/s]
        self.max_y_velocity = 0.015  # [m/s]
        self.max_yaw_rate = 0.6      # [rad/s]
        
        # Scale factors to normalize cmd_vel values to joystick range [-1, 1]
        self.linear_x_scale = 1.0 / self.max_x_velocity
        self.linear_y_scale = 1.0 / self.max_y_velocity
        self.angular_z_scale = 1.0 / self.max_yaw_rate
        
        # Subscribe to cmd_vel topic
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher to send joystick-like commands to the robot controller
        self.joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)
        
        # Initialize mode
        self.mode_set = False
        self.lqr_enabled = False
        self.last_cmd_time = rospy.Time.now()
        self.timeout = rospy.Duration(1.0)  # 1 second timeout
        
        rospy.loginfo("CmdVelToJoints node initialized with trot gait parameters")
        
        # Timer for keeping the robot in the right mode
        rospy.Timer(rospy.Duration(1.0), self.mode_timer_callback)
        
        rospy.spin()
    
    def mode_timer_callback(self, event):
        """Ensure the robot stays in the correct mode"""
        if not self.mode_set or not self.lqr_enabled:
            self.send_mode_command()
    
    def send_mode_command(self):
        """Set the robot to trot mode with LQR control initially"""
        # First put robot in trot mode
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Button 1 is trot mode
        joy_msg.buttons[1] = 1
        
        self.joy_pub.publish(joy_msg)
        rospy.loginfo("Setting robot to TROT mode")
        time.sleep(0.5)  # Wait for mode to take effect
        self.mode_set = True
        
        # Then enable LQR control
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Button 9 is LQR control
        joy_msg.buttons[9] = 1
        
        self.joy_pub.publish(joy_msg)
        rospy.loginfo("Enabling LQR control for stability")
        time.sleep(0.5)  # Wait for LQR to activate
        self.lqr_enabled = True
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to Joy message"""
        # Check if we need to set the mode first
        if not self.mode_set or not self.lqr_enabled:
            self.send_mode_command()
        
        # Create Joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Map cmd_vel to joystick axes with proper scaling
        # For the NotSpot trot controller:
        # - Axis 0: Left/right rotation (angular.z)
        # - Axis 2: Left/right movement (linear.y)
        # - Axis 3: Forward/backward (linear.x)
        
        # Scale velocities to joystick range [-1, 1]
        # Apply scaling factors and clamp to [-1, 1] range
        angular_z = max(min(msg.angular.z * self.angular_z_scale, 1.0), -1.0)
        linear_y = max(min(msg.linear.y * self.linear_y_scale, 1.0), -1.0)
        linear_x = max(min(msg.linear.x * self.linear_x_scale, 1.0), -1.0)
        
        joy_msg.axes[0] = angular_z  # Rotation
        joy_msg.axes[2] = linear_y   # Left/right movement
        joy_msg.axes[3] = linear_x   # Forward/backward
        
        # Update last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Publish the joy message
        self.joy_pub.publish(joy_msg)
        
        # Log periodic status (avoiding spamming the log)
        if rospy.Time.now().to_sec() % 5 < 0.1:  # Log approximately every 5 seconds
            rospy.loginfo(f"CMD_VEL to Joy: lin_x: {linear_x:.2f}, lin_y: {linear_y:.2f}, ang_z: {angular_z:.2f}")

if __name__ == '__main__':
    try:
        CmdVelToJoints()
    except rospy.ROSInterruptException:
        pass