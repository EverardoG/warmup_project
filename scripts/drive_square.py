#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

import tty
import select
import sys
import termios

class DriveSquareNode(object):
    def __init__(self):
        rospy.init_node('drive_square')
        rospy.Subscriber('/odom', Odometry, self.callback_func)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None

        self.start = False
        self.active = False
        self.delta_distance = 0.0
        self.init_pos = np.array([None, None]) # [x, y]
        self.current_pos = np.array([None, None]) # [x, y]

        self.dist_edge = 2.0 # meters
        # self.dist_travelled = 0.0 # meters

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)

        tty.setraw(sys.stdin.fileno())

        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def callback_func(self, msg):
        # current position
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1]= msg.pose.pose.position.y

    def run(self):
        r = rospy.Rate(10)
        print("Initialized node.")
        while not rospy.is_shutdown():
            self.key = self.getKey()
            if self.key == '\x03': print ("Shutting down node"); break

            # toggle start of square
            if not self.start and self.key == "\r":
                self.start = True
                self.active = True
                self.init_pos = self.current_pos
                print("Robot is starting square.")
            # toggle whether robot is active
            if self.start and self.key == " ":
                self.active = not self.active
                if self.active: print("Activating Robot.")
                else: print("Stopping Robot.")

            # flight code below
            print(self.start)
            print(self.active)
            if self.start and self.active:
                twist_msg = Twist()

                dist_travelled = np.linalg.norm(self.current_pos - self.init_pos)
                if dist_travelled < self.dist_edge:
                    twist_msg.linear.x = 0.3
                else:
                    twist_msg.linear.x = 0.0

                print("dist_travelled: ", dist_travelled)
                print("dist_edge: ", self.dist_edge)
                self.pub.publish(twist_msg)

                # check current distance travelled
                # if it's less than the desired distance
                # then keep going
                # if it's greater, then stop
            elif not self.active:
                self.pub.publish(Twist())

            # if not self.active and self.getKey() == " ":
            #     self.active = True
            # elif sel

            # self.pub.publish(MsgType(linear=Vector3(x=self.desired_velocity)))
            r.sleep()

        # if self.key == '\x03': print("Shutting down node.")



if __name__ == '__main__':
    drivesquare = DriveSquareNode()
    drivesquare.run()
