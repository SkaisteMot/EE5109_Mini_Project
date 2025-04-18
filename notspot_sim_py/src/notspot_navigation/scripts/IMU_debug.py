#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # Extract orientation quaternion
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    # Print orientation data
    rospy.loginfo("IMU Orientation:")
    rospy.loginfo("  Quaternion: x=%.4f, y=%.4f, z=%.4f, w=%.4f", 
                 msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    rospy.loginfo("  Euler angles (radians): roll=%.4f, pitch=%.4f, yaw=%.4f", 
                 roll, pitch, yaw)
    rospy.loginfo("  Euler angles (degrees): roll=%.2f°, pitch=%.2f°, yaw=%.2f°", 
                 roll*180/3.14159, pitch*180/3.14159, yaw*180/3.14159)
    
    # Print angular velocity
    rospy.loginfo("Angular Velocity:")
    rospy.loginfo("  x=%.4f, y=%.4f, z=%.4f", 
                 msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    
    # Print linear acceleration
    rospy.loginfo("Linear Acceleration:")
    rospy.loginfo("  x=%.4f, y=%.4f, z=%.4f", 
                 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    
    rospy.loginfo("----------------")

def imu_debug_node():
    rospy.init_node('imu_debug_node', anonymous=True)
    
    # Subscribe to the IMU topic
    rospy.Subscriber('/notspot_imu/base_link_orientation', Imu, imu_callback)
    
    # Keep the node running
    rospy.loginfo("IMU Debug Node started. Monitoring /notspot_imu/base_link_orientation...")
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_debug_node()
    except rospy.ROSInterruptException:
        pass