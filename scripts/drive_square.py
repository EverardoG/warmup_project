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

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callback_func(self, msg):
        # current position
        # print("callback function called")
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1]= msg.pose.pose.position.y

    def run(self):
        r = rospy.Rate(10)
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        while not rospy.is_shutdown():
            # print("running main loop")
            self.key = None
            if self.keyWasPressed():
                self.key = sys.stdin.read(1)
            if self.key == '\x03': print ("Shutting down node\r"); break
            # toggle start of square
            if not self.start and self.key == "\r":
                self.start = True
                self.active = True
                self.init_pos = self.current_pos.copy()
                print("Robot is starting square.\r")
            # toggle whether robot is active
            if self.start and self.key == " ":
                self.active = not self.active
                if self.active: print("Continuing Robot.\r")
                else: print("Pausing Robot.\r")

            if self.start and self.active:
                twist_msg = Twist()
                # print("Current Pos x: ", self.current_pos[0], " | y: ", self.current_pos[1],'\r')
                # print("Init Pos x: ", self.init_pos[0], " | y: ", self.init_pos[1],'\r')
                dist_travelled = np.linalg.norm(self.current_pos - self.init_pos)
                if dist_travelled < self.dist_edge:
                    twist_msg.linear.x = 0.3
                else:
                    twist_msg.linear.x = 0.0
                    print("Finished edge.\r")

                # print("dist_travelled: ", dist_travelled)
                # print("dist_edge: ", self.dist_edge)
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # if self.key == '\x03': print("Shutting down node.")



if __name__ == '__main__':
    drivesquare = DriveSquareNode()
    drivesquare.run()
