#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoints:
    def __init__(self):
        rospy.init_node('cmd_vel_to_joints')
        
        # Parameters
        self.auto_mode = rospy.get_param('~auto_mode', 'trot')  # Default gait mode
        self.linear_scale = rospy.get_param('~linear_scale', 0.5)  # Scale for linear velocity
        self.angular_scale = rospy.get_param('~angular_scale', 0.5)  # Scale for angular velocity
        
        # Subscribe to cmd_vel topic
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher to send joystick-like commands to the robot controller
        self.joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)
        
        # Initialize mode
        self.mode_set = False
        self.last_cmd_time = rospy.Time.now()
        self.timeout = rospy.Duration(1.0)  # 1 second timeout
        
        rospy.loginfo("CmdVelToJoints node initialized with {} mode".format(self.auto_mode))
        
        # Timer for keeping the robot in the right mode
        rospy.Timer(rospy.Duration(1.0), self.mode_timer_callback)
        
        rospy.spin()
    
    def mode_timer_callback(self, event):
        """Ensure the robot stays in the correct mode"""
        if not self.mode_set:
            self.send_mode_command()
    
    def send_mode_command(self):
        """Send initial mode command to the robot"""
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Set mode based on parameter
        if self.auto_mode == 'trot':
            joy_msg.buttons[1] = 1  # Trot mode
            rospy.loginfo("Setting robot to TROT mode")
        elif self.auto_mode == 'crawl':
            joy_msg.buttons[2] = 1  # Crawl mode
            rospy.loginfo("Setting robot to CRAWL mode")
        elif self.auto_mode == 'stand':
            joy_msg.buttons[3] = 1  # Stand mode
            rospy.loginfo("Setting robot to STAND mode")
        else:
            joy_msg.buttons[0] = 1  # Rest mode
            rospy.loginfo("Setting robot to REST mode")
        
        self.joy_pub.publish(joy_msg)
        time.sleep(0.5)  # Wait to ensure mode change takes effect
        self.mode_set = True
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to Joy message"""
        # Check if we need to set the mode first
        if not self.mode_set:
            self.send_mode_command()
        
        # Create Joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Map cmd_vel to joystick axes
        # The mapping is based on the NotSpot control scheme:
        # - Axis 0: Left/right rotation (angular.z)
        # - Axis 3: Forward/backward (linear.x)
        # - Axis 2: Left/right movement (linear.y)
        
        joy_msg.axes[0] = msg.angular.z * self.angular_scale  # Rotation
        joy_msg.axes[3] = msg.linear.x * self.linear_scale    # Forward/backward
        joy_msg.axes[2] = msg.linear.y * self.linear_scale    # Left/right
        
        # Update last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Publish the joy message
        self.joy_pub.publish(joy_msg)
        
        # Log periodic status (avoiding spamming the log)
        if rospy.Time.now().to_sec() % 5 < 0.1:  # Log approximately every 5 seconds
            rospy.loginfo(f"Converting cmd_vel - lin: [{msg.linear.x:.2f}, {msg.linear.y:.2f}], ang: {msg.angular.z:.2f}")

if __name__ == '__main__':
    try:
        CmdVelToJoints()
    except rospy.ROSInterruptException:
        pass