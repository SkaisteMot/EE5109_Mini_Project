#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoints:
    def __init__(self):
        rospy.init_node('cmd_vel_to_joints')
        
        # Get parameters
        self.auto_mode = rospy.get_param('~auto_mode', 'trot')  # Default to trot
        
        # Parameters matching trot gait controller speeds from controller code
        self.max_x_velocity = 0.024  # [m/s] from TrotGaitController.py
        self.max_y_velocity = 0.015  # [m/s] from TrotGaitController.py
        self.max_yaw_rate = 0.6      # [rad/s] from TrotGaitController.py
        
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
        self.stabilize_enabled = False
        
        # Keep track of last command time for timeout safety
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(1.0)  # 1 second timeout
        
        rospy.loginfo(f"CmdVelToJoints node initialized with {self.auto_mode} gait parameters")
        
        # Timer for keeping the robot in the right mode
        rospy.Timer(rospy.Duration(1.0), self.mode_timer_callback)
        
        # Safety timer to stop robot if no commands are received
        rospy.Timer(rospy.Duration(0.5), self.safety_timer_callback)
        
    def safety_timer_callback(self, event):
        """Stop the robot if no commands have been received recently"""
        if (rospy.Time.now() - self.last_cmd_time) > self.cmd_timeout:
            # Send zero velocity command
            joy_msg = Joy()
            joy_msg.axes = [0.0] * 8
            joy_msg.buttons = [0] * 12
            self.joy_pub.publish(joy_msg)
    
    def mode_timer_callback(self, event):
        """Ensure the robot stays in the correct mode"""
        if not self.mode_set:
            self.send_mode_command()
            
        # Re-enable stability control periodically
        if not self.stabilize_enabled:
            self.enable_stability_control()
    
    def send_mode_command(self):
        """Set the robot to the appropriate mode (trot, crawl, etc.)"""
        # Create basic joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Set the appropriate button based on mode
        if self.auto_mode == 'trot':
            # Button 1 is trot mode
            joy_msg.buttons[1] = 1
            rospy.loginfo("Setting robot to TROT mode")
        elif self.auto_mode == 'crawl':
            # Button 2 is crawl mode
            joy_msg.buttons[2] = 1
            rospy.loginfo("Setting robot to CRAWL mode")
        elif self.auto_mode == 'stand':
            # Button 3 is stand mode
            joy_msg.buttons[3] = 1
            rospy.loginfo("Setting robot to STAND mode")
        
        # Publish the message
        self.joy_pub.publish(joy_msg)
        time.sleep(0.5)  # Wait for mode to take effect
        self.mode_set = True
    
    def enable_stability_control(self):
        """Enable LQR control for better stability"""
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Enable LQR control - Button 9 from TrotGaitController.py
        joy_msg.buttons[9] = 1
        
        self.joy_pub.publish(joy_msg)
        rospy.loginfo("Enabling LQR stability control")
        time.sleep(0.2)  # Short pause
        self.stabilize_enabled = True
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to Joy message"""
        # Check if we need to set the mode first
        if not self.mode_set:
            self.send_mode_command()
            self.enable_stability_control()
        
        # Create Joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Map cmd_vel to joystick axes with proper scaling based on TrotGaitController.py
        # For the NotSpot trot controller:
        # - Axis 2: Left/right rotation (angular.z - yaw_rate)
        # - Axis 0: Left/right movement (linear.y)
        # - Axis 3: Forward/backward (linear.x)
        
        # Scale velocities to joystick range [-1, 1]
        # Apply scaling factors and clamp to [-1, 1] range
        angular_z = max(min(msg.angular.z * self.angular_z_scale, 1.0), -1.0)
        linear_y = max(min(msg.linear.y * self.linear_y_scale, 1.0), -1.0)
        linear_x = max(min(msg.linear.x * self.linear_x_scale, 1.0), -1.0)
        
        # Apply the values to the correct axes based on TrotGaitController.updateStateCommand
        joy_msg.axes[2] = angular_z   # Rotation (yaw_rate)
        joy_msg.axes[0] = linear_y    # Left/right (lateral)
        joy_msg.axes[3] = linear_x    # Forward/backward (straight)
        
        # Update last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Publish the joy message
        self.joy_pub.publish(joy_msg)
        
        # Log periodic status (avoiding spamming the log)
        if int(rospy.Time.now().to_sec()) % 5 == 0:  # Log approximately every 5 seconds
            rospy.loginfo(f"CMD_VEL converted: lin_x: {linear_x:.2f}, lin_y: {linear_y:.2f}, ang_z: {angular_z:.2f}")

if __name__ == '__main__':
    try:
        CmdVelToJoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass