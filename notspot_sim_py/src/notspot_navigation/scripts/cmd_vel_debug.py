#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    rospy.loginfo("CMD_VEL Command:")
    rospy.loginfo("  Linear: x=%.4f, y=%.4f, z=%.4f", 
                 msg.linear.x, msg.linear.y, msg.linear.z)
    rospy.loginfo("  Angular: x=%.4f, y=%.4f, z=%.4f", 
                 msg.angular.x, msg.angular.y, msg.angular.z)
    rospy.loginfo("----------------")

def cmd_vel_debug_node():
    rospy.init_node('cmd_vel_debug_node', anonymous=True)
    
    # Subscribe to cmd_vel topic
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    # Keep the node running
    rospy.loginfo("CMD_VEL Debug Node started. Monitoring /cmd_vel...")
    rospy.spin()

if __name__ == '__main__':
    try:
        cmd_vel_debug_node()
    except rospy.ROSInterruptException:
        pass