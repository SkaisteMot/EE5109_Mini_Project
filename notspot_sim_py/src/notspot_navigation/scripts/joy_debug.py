#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def joy_callback(msg):
    rospy.loginfo("Joy Command:")
    
    # Print axes values with labels
    axes_labels = ["Lateral (0)", "Height (1)", "Rotation (2)", "Forward (3)", 
                  "Axis 4", "Axis 5", "Axis 6", "Axis 7"]
    
    rospy.loginfo("  Axes:")
    for i, value in enumerate(msg.axes):
        if i < len(axes_labels):
            rospy.loginfo("    %s: %.4f", axes_labels[i], value)
        else:
            rospy.loginfo("    Axis %d: %.4f", i, value)
    
    # Print active buttons
    active_buttons = []
    for i, value in enumerate(msg.buttons):
        if value > 0:
            active_buttons.append(i)
    
    if active_buttons:
        rospy.loginfo("  Active Buttons: %s", str(active_buttons))
    else:
        rospy.loginfo("  No Active Buttons")
    
    rospy.loginfo("----------------")

def joy_debug_node():
    rospy.init_node('joy_debug_node', anonymous=True)
    
    # Subscribe to joy topic
    rospy.Subscriber('/notspot_joy/joy_ramped', Joy, joy_callback)
    
    # Keep the node running
    rospy.loginfo("Joy Debug Node started. Monitoring /notspot_joy/joy_ramped...")
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_debug_node()
    except rospy.ROSInterruptException:
        pass