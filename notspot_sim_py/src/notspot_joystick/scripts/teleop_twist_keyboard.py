#!/usr/bin/env python3

from __future__ import print_function
import rospy
from sensor_msgs.msg import Joy
import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to Joy!
---------------------------
Movement:
   w : Forward (Axis 1)
   s : Backward (Axis 1)
   a : Left (Axis 0)
   d : Right (Axis 0)

Rotation and Height:
   i : Rotate left (Axis 2)
   k : Rotate right (Axis 2)
   j : Increase height (Axis 4)
   l : Decrease height (Axis 4)

Buttons (Number Keys):
   1 : Rest Controller
   2 : Trot Gait Controller
   3 : Crawl Gait Controller
   4 : Stand Controller
   5 : Button 4
   6 : Button 5
   7 : Button 6
   8 : Button 7
   9 : LQR Control
   x : Stop

CTRL-C to quit
"""

# Key bindings for movement, rotation, and height
moveBindings = {
    'w': (0, 0, 0, 1, 0, 0, 0, 0),     # Forward (Axis 4)
    's': (0, 0, 0, -1, 0, 0, 0, 0),    # Backward (Axis 4)
    'a': (1, 0, 0, 0, 0, 0, 0, 0),     # Left (Axis 0)
    'd': (-1, 0, 0, 0, 0, 0, 0, 0),    # Right (Axis 0)
    'i': (0, 1, 0, 0, 0, 0, 0, 0),     # Rotate left (Axis 1)
    'k': (0, -1, 0, 0, 0, 0, 0, 0),    # Rotate right (Axis 1)
    'j': (0, 0, 1, 0, 0, 0, 0, 0),     # Height up (Axis 2)
    'l': (0, 0, -1, 0, 0, 0, 0, 0),    # Height down (Axis 2)
    't': (0, 0, 0, 0, 0, 0, 1, 0),     # Custom (Axis 6)
    'g': (0, 0, 0, 0, 0, 0, -1, 0),    # Custom (Axis 6)
    'f': (0, 0, 0, 0, 0, 0, 0, 1),     # Custom (Axis 7)
    'h': (0, 0, 0, 0, 0, 0, 0, -1),    # Custom (Axis 7)
}

# Button bindings for number keys
buttonBindings = {
    '1': 0,  # Rest Controller
    '2': 1,  # Trot Gait Controller
    '3': 2,  # Crawl Gait Controller
    '4': 3,  # Stand Controller
    '5': 4,  # Button 4
    '6': 5,  # Button 5
    '7': 6,  # Stop - (Trot ONLY)
    '8': 7,  # PID Control
    'x': 8,  # Stop Command
    '9': 9,  # LQR Control

}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    joy_pub = rospy.Publisher('/notspot_joy/joy_ramped', Joy, queue_size=1)

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            joy = Joy()
            joy.axes = [0.0] * 8  # Initialize with 8 axes
            joy.buttons = [0] * 12  # Initialize with 12 buttons

            if key in moveBindings.keys():
                values = moveBindings[key]

                if len(joy.axes) < 8:
                    joy.axes = [0.0] * 8
                
                # Assign all values to the corresponding axes
                for i in range(len(values)):
                    if i < len(joy.axes):
                        joy.axes[i] = values[i]

            elif key in buttonBindings.keys():
                # Map number keys to buttons
                button_index = buttonBindings[key]
                joy.buttons[button_index] = 1  # Press the button

            elif key == '\x03':  # CTRL-C
                break

            # Publish the Joy message
            joy_pub.publish(joy)

    except Exception as e:
        print(e)

    finally:
        # Reset the Joy message
        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 12
        joy_pub.publish(joy)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
