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
        self.max_yaw_rate = 0.06      # [rad/s] from TrotGaitController.py 
        
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
        
        # For direct axis inspection
        self.joy_sub = rospy.Subscriber('/notspot_joy/joy_ramped', Joy, self.joy_callback)
        self.last_joy_msg = None
        
    def joy_callback(self, msg):
        """Store joy message for debugging"""
        self.last_joy_msg = msg
        
        
    def safety_timer_callback(self, event):
        """Stop the robot if no commands have been received recently"""
        if (rospy.Time.now() - self.last_cmd_time) > self.cmd_timeout:
            # Send zero velocity command
            joy_msg = Joy()
            joy_msg.axes = [0.0] * 8
            joy_msg.buttons = [0] * 12
            self.joy_pub.publish(joy_msg)
            rospy.logdebug("No cmd_vel received recently - stopping robot")
    
    def mode_timer_callback(self, event):
        """Ensure the robot stays in the correct mode"""
        if not self.mode_set:
            self.send_mode_command()
            
        # Re-enable stability control periodically
        if not self.stabilize_enabled:
            self.enable_stability_control()
    
    def send_mode_command(self):
        """Set the robot to the appropriate mode (trot, crawl, etc.) with proper timing"""
        # First, ensure clean state by sending rest mode
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        joy_msg.buttons[1] = 1  # Trot Mode
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.7)  # Wait for mode to take effect
        
        # Clear buttons
        joy_msg.buttons = [0] * 12
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.3)
        
        # Set the appropriate button based on mode
        if self.auto_mode == 'trot':
            joy_msg.buttons[1] = 1
            rospy.loginfo("Setting robot to TROT mode")
        elif self.auto_mode == 'crawl':
            joy_msg.buttons[2] = 1
            rospy.loginfo("Setting robot to CRAWL mode")
        elif self.auto_mode == 'stand':
            joy_msg.buttons[3] = 1
            rospy.loginfo("Setting robot to STAND mode")
        
        # Publish the message
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.7)  # Wait for mode to take effect
        
        # Clear buttons again
        joy_msg.buttons = [0] * 12
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.3)
        
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
        rospy.sleep(0.5)  # Longer pause to ensure it takes effect
        
        # Clear button press
        joy_msg.buttons = [0] * 12
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.2)
        
        self.stabilize_enabled = True # Enable LQR
    
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
        
        # Log input values
        rospy.logdebug(f"CMD_VEL Input: linear_x={msg.linear.x:.4f}, linear_y={msg.linear.y:.4f}, angular_z={msg.angular.z:.4f}")
        
        # Scale velocities to joystick range [-1, 1]
        angular_z = max(min(msg.angular.z * self.angular_z_scale, 1.0), -1.0)
        linear_y = max(min(msg.linear.y * self.linear_y_scale, 1.0), -1.0)
        linear_x = max(min(msg.linear.x * self.linear_x_scale, 1.0), -1.0)
        
        # Log scaled values
        rospy.logdebug(f"Scaled values: linear_x={linear_x:.4f}, linear_y={linear_y:.4f}, angular_z={angular_z:.4f}")
        
        # STANDARD MAPPING (Should match TrotGaitController.updateStateCommand)
        joy_msg.axes[3] = linear_x    # Forward/backward on axis 3 
        joy_msg.axes[0] = linear_y    # Left/right on axis 0  
        joy_msg.axes[2] = angular_z   # Rotation on axis 2
        
        # Update last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Publish the joy message
        self.joy_pub.publish(joy_msg)
        
        # Log detailed values every time for debugging
        rospy.loginfo(f"CMD_VEL to JOY: x={linear_x:.3f}→axis3={joy_msg.axes[3]:.3f}, y={linear_y:.3f}→axis0={joy_msg.axes[0]:.3f}, θ={angular_z:.3f}→axis2={joy_msg.axes[2]:.3f}")

if __name__ == '__main__':
    try:
        CmdVelToJoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass