#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
from scipy.spatial.transform import Rotation as R

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

        self.init_angle = None
        self.current_angle = None

        self.dist_edge = 1.0 # meters
        self.num_edge = 1
        self.state = "line" # or "corner"

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callback_func(self, msg):
        # current position
        # print("callback function called")
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        # self.current_angle  = msg.pose.pose.orientation.z

        r = R.from_quat([0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        euler_angles = r.as_euler('xyz', degrees=True)
        self.current_angle = euler_angles[2]
        # print(self.current_angle, '\r')

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

            # flight code for driving in square
            if self.start and self.active:
                twist_msg = Twist()

                # Drive straight
                if self.state == "line":
                    # print("Current Pos x: ", self.current_pos[0], " | y: ", self.current_pos[1],'\r')
                    # print("Init Pos x: ", self.init_pos[0], " | y: ", self.init_pos[1],'\r')
                    dist_travelled = np.linalg.norm(self.current_pos - self.init_pos)
                    print(dist_travelled,"\r")

                    # Run proportional control
                    if dist_travelled < self.dist_edge:
                        twist_msg.linear.x = 1.5 * (self.dist_edge-dist_travelled) + 0.001
                        if twist_msg.linear.x > 0.3:
                            twist_msg.linear.x = 0.3

                    else:
                        twist_msg.linear.x = 0.0
                        if self.num_edge != 4:
                            print("Finished edge number",self.num_edge,"\r")
                            self.num_edge+=1
                            self.init_angle = self.current_angle
                            self.state = "corner"
                        else:
                            print("Finished the square!\r")

                # Turn
                elif self.state == "corner":

                    # Handle edge case where robot turns from 180 to -180
                    if self.current_angle < self.init_angle and (self.current_angle - self.init_angle) < -0.01:
                        angle_travelled = (180 - self.init_angle) + (180 + self.current_angle)
                    else:
                        angle_travelled = self.current_angle - self.init_angle

                    # print(self.current_angle,'\r')
                    print("angle info\r")
                    print("Initial Angle: ", self.init_angle,'\r')
                    print("Current Angle: ", self.current_angle,'\r')
                    print("Angle Travelled: ", angle_travelled,'\r')

                    # Run proportional control
                    if angle_travelled < 90.0:
                        twist_msg.angular.z = 0.008 * (90.0 - angle_travelled) + 0.01
                        if twist_msg.angular.z > 1.0:
                            twist_msg.angular.z = 1.0
                    else:
                        print("Finished corner number", self.num_edge,'\r')
                        self.init_pos = self.current_pos.copy()
                        self.state = "line"

                    # print(angle_travelled,'\r')
                    # twist_msg.angular.z = 1.0


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
    drivesquare = DriveSquareNode()
    drivesquare.run()
