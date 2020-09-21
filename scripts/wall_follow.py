#!/usr/bin/env python3

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


import numpy as np
# from scipy.spatial.transform import Rotation as R

import tty
import select
import sys
import termios

class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.callback_func)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None

        self.start = False
        self.active = False

        self.point_1 = None
        self.point_2 = None

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callback_func(self, msg):
        # update both points
        self.point_1 = msg.ranges[315]
        self.point_2 = msg.ranges[225]

    def run(self):
        r = rospy.Rate(10)
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run\r")
        while not rospy.is_shutdown():
            self.key = None
            if self.keyWasPressed():
                self.key = sys.stdin.read(1)
            if self.key == '\x03': print ("Shutting down node\r"); break
            # toggle start of program
            if not self.start and self.key == "\r":
                self.start = True
                self.active = True
                print("Robot is starting wall follow mode.\r")
            # toggle whether robot is active
            if self.start and self.key == " ":
                self.active = not self.active
                if self.active: print("Continuing Robot.\r")
                else: print("Pausing Robot.\r")

            # flight code for wall following
            if self.start and self.active:
                twist_msg = Twist()

                # Check the difference between the two points
                # turn either left or right proportionally dependent on the difference
                # fixed forward motion
                if np.isinf(self.point_1): self.point_1 = 1000
                if np.isinf(self.point_2): self.point_2 = 1000

                print(repr(self.point_1), ' | ', repr(self.point_2),'\r')
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = - (self.point_1 - self.point_2)

                if twist_msg.angular.z > 1.0: twist_msg.angular.z = 1.0
                elif twist_msg.angular.z < -1.0: twist_msg.angular.z = -1.0

                self.pub.publish(twist_msg)

            elif not self.active:
                self.pub.publish(Twist())

            # if not self.active and self.getKey() == " ":
            #     self.active = True
            # elif sel

            # self.pub.publish(MsgType(linear=Vector3(x=self.desired_velocity)))
            r.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # if self.key == '\x03': print("Shutting down node.")



if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
