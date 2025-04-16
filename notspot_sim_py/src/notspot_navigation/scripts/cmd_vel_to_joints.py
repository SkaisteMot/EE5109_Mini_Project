import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoints:
    def __init__(self):
        rospy.init_node('cmd_vel_to_joints')
        
        # Subscribe to cmd_vel topic
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher to send joystick-like commands to your robot controller
        self.joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)
        
        rospy.spin()
    
    def cmd_vel_callback(self, msg):
        # Convert Twist message to Joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        
        # Set axes based on cmd_vel
        # You'll need to adjust these mappings based on your robot's control scheme
        joy_msg.axes[0] = msg.angular.z * 0.5  # Steering
        joy_msg.axes[3] = msg.linear.x * 0.5   # Forward/backward
        
        # Set to trot mode if needed
        if not hasattr(self, 'mode_set'):
            joy_msg.buttons[1] = 1  # Trot mode
            self.mode_set = True
        
        self.joy_pub.publish(joy_msg)

if __name__ == '__main__':
    try:
        CmdVelToJoints()
    except rospy.ROSInterruptException:
        pass