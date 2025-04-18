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
        
        #rospy.loginfo(f"CmdVelToJoints node initialized with {self.auto_mode} gait parameters")
        #rospy.loginfo(f"* MAXIMUM VALUES - X: {self.max_x_velocity} m/s, Y: {self.max_y_velocity} m/s, YAW: {self.max_yaw_rate} rad/s")
        #rospy.loginfo(f"* SCALE FACTORS - X: {self.linear_x_scale}, Y: {self.linear_y_scale}, YAW: {self.angular_z_scale}")
        
        # Timer for keeping the robot in the right mode
        rospy.Timer(rospy.Duration(1.0), self.mode_timer_callback)
        
        # Safety timer to stop robot if no commands are received
        rospy.Timer(rospy.Duration(0.5), self.safety_timer_callback)
        
        # Debugging timer to print current control state
        rospy.Timer(rospy.Duration(2.0), self.debug_timer_callback)
        
    def joy_callback(self, msg):
        """Store joy message for debugging"""
        self.last_joy_msg = msg
        
    def debug_timer_callback(self, event):
        """Regularly print debug info about control state"""
        if self.last_joy_msg:
            #rospy.loginfo("=== JOYSTICK STATUS ===")
            #rospy.loginfo(f"Mode: {self.auto_mode}, Mode set: {self.mode_set}, LQR: {self.stabilize_enabled}")
            #rospy.loginfo(f"Axes values:")
            
            # These are the specific axes we care about
            axis_names = ["lateral (0)", "height (1)", "rotation (2)", "forward (3)", 
                          "axis4", "axis5", "axis6", "axis7"]
            
            for i, name in enumerate(axis_names):
                if i < len(self.last_joy_msg.axes):
                    rospy.loginfo(f"  {name}: {self.last_joy_msg.axes[i]:.3f}")
            
            # Print active buttons
            active_buttons = []
            for i, val in enumerate(self.last_joy_msg.buttons):
                if val > 0:
                    active_buttons.append(i)
            
            if active_buttons:
                rospy.loginfo(f"Active buttons: {active_buttons}")
            else:
                rospy.loginfo("No active buttons")
                
            #rospy.loginfo("=====================")
        
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
        rospy.loginfo("=== SETTING ROBOT MODE ===")
        # First, ensure clean state by sending rest mode
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        joy_msg.buttons[0] = 1  # Rest mode
        self.joy_pub.publish(joy_msg)
        rospy.loginfo("Sent REST mode command")
        rospy.sleep(0.7)  # Wait for mode to take effect
        
        # Clear buttons
        joy_msg.buttons = [0] * 12
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.3)
        
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
        rospy.sleep(0.7)  # Wait for mode to take effect
        
        # Clear buttons again
        joy_msg.buttons = [0] * 12
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.3)
        
        self.mode_set = True
        rospy.loginfo("Mode setup complete")
    
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
        
        self.stabilize_enabled = True
        rospy.loginfo("LQR control enabled")
    
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
        # Apply scaling factors and clamp to [-1, 1] range
        angular_z = max(min(msg.angular.z * self.angular_z_scale, 1.0), -1.0)
        linear_y = max(min(msg.linear.y * self.linear_y_scale, 1.0), -1.0)
        linear_x = max(min(msg.linear.x * self.linear_x_scale, 1.0), -1.0)
        
        # Log scaled values
        rospy.logdebug(f"Scaled values: linear_x={linear_x:.4f}, linear_y={linear_y:.4f}, angular_z={angular_z:.4f}")
        
        # INVESTIGATE ALTERNATIVE MAPPINGS
        # Try different axis mappings - TrotGaitController.py:
        # self.target_joy.axes = msg.axes
        # command.velocity[0] = msg.axes[3] * self.max_x_velocity # Straight - AXIS 3
        # command.velocity[1] = msg.axes[0] * self.max_y_velocity # Lateral - AXIS 0
        # command.yaw_rate = msg.axes[2] * self.max_yaw_rate      # Rotation - AXIS 2
        
        # STANDARD MAPPING (Should match TrotGaitController.updateStateCommand)
        joy_msg.axes[2] = -linear_x    # Forward/backward on axis 3 
        joy_msg.axes[1] = linear_y    # Left/right on axis 0  
        joy_msg.axes[3] = angular_z   # Rotation on axis 2
        
        # Boost linear movement signal to overcome friction
        if abs(linear_x) > 0.1:  # If there's a significant forward command
            # Boost it slightly to overcome inertia
            joy_msg.axes[3] = linear_x * 1.5
            if joy_msg.axes[3] > 1.0:
                joy_msg.axes[3] = 1.0
            elif joy_msg.axes[3] < -1.0:
                joy_msg.axes[3] = -1.0
                
            # Reduce yaw to prioritize forward motion
            joy_msg.axes[2] = angular_z * 0.7
        
        # Additional axes value to ensure robot is in proper height state
        joy_msg.axes[1] = 0.0         # Height control
        
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