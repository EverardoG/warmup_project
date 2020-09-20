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
        self.des_linear_vel = 0.0
        self.des_angular_vel = 0.0

    def run(self):
        r = rospy.Rate(10)
        # This loop only seems to run when a key is pressed
        # AND the correct terminal window is selected
        while not rospy.is_shutdown() and self.key != '\x03':
            self.key = self.getKey()
            print("self.key: ", repr(self.key))
            twist_msg = self.keyToTwist()

            if twist_msg is not None:
                print("linear vel: ", self.des_linear_vel)
                print("angular vel: ", self.des_angular_vel)
                self.pub.publish(twist_msg)
            r.sleep()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyToTwist(self):
        # Forward
        if self.key == "w":
            self.des_linear_vel += 0.1
        # Backward
        elif self.key == "s":
            self.des_linear_vel -= 0.1
        # Right
        elif self.key == "d":
            if self.des_angular_vel > 0:
                self.des_angular_vel = 0
            else:
                self.des_angular_vel -= 0.1
        # Left
        elif self.key == "a":
            if self.des_angular_vel < 0:
                self.des_angular_vel = 0
            else:
                self.des_angular_vel += 0.1
        # ESTOP
        elif self.key == " ":
            self.des_linear_vel = 0.0
            self.des_angular_vel = 0.0
        # Invalid
        else:
            return None

        # Linear velocity limits
        if self.des_linear_vel > 0.5:
            self.des_linear_vel = 0.5
        elif self.des_linear_vel < -0.5:
            self.des_linear_vel = -0.5

        # Angular velocity limits
        if self.des_angular_vel > 1.0:
            self.des_angular_vel = 1.0
        elif self.des_angular_vel < -1.0:
            self.des_angular_vel = -1.0


        twist_msg = Twist(linear=Vector3(x=self.des_linear_vel), angular=Vector3(z=self.des_angular_vel))
        return twist_msg

settings = termios.tcgetattr(sys.stdin)

if __name__ == '__main__':
    teleop = CustomTeleopNode()
    teleop.run()
