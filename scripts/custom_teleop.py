#!/usr/bin/env python3

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

class CustomTeleopNode(object):
    def __init__(self):
        rospy.init_node('custom_teleop_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None
        # self.desired_velocity = 0.3

    def run(self):
        r = rospy.Rate(10)
        # This loop only seems to run when a key is pressed
        # AND the correct terminal window is selected
        while not rospy.is_shutdown() and self.key != '\x03':
            self.key = self.getKey()
            print("self.key: ", repr(self.key))
            twist_msg = self.keyToTwist()

            if twist_msg is not None:
                self.pub.publish(twist_msg)
            r.sleep()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyToTwist(self):
        des_x = None
        if self.key == "w":
            des_x = 0.3
        elif self.key == " ":
            des_x = 0.0

        if des_x is None:
            return None
        else:
            twist_msg = Twist(linear=Vector3(x=des_x))
            return twist_msg

settings = termios.tcgetattr(sys.stdin)
# key = None

# while key != '\x03':
#     key = getKey()
#     print("got key:", key)

if __name__ == '__main__':
    teleop = CustomTeleopNode()
    teleop.run()
