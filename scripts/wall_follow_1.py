#!/usr/bin/env python3

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


import numpy as np
Polynomial = np.polynomial.Polynomial
# from scipy.spatial.transform import Rotation as R

import tty

import select
import sys
import termios

# 🐛 The Worm is Watching
class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.callbackFunc)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None

        self.start = False
        self.active = False

        # self.point_1 = None
        # self.point_2 = None

        self.scan_angle = 270 # 270 for right wall, 90 for left
        self.scan_range = 30

        self.max_angular_speed = 1.0

        # Make data containers easily accessible for later
        self.points_r_theta = None # np.zeros((2, self.scan_range))
        self.points_x_y = None # np.zeros((2, self.scan_range))
        self.desired_angle = None # wall angle

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callbackFunc(self, msg):
        # print("Running callback \r")
        # # update both points
        # self.point_1 = msg.ranges[315]
        # self.point_2 = msg.ranges[225]

        # update point data
        start_angle = int(self.scan_angle-self.scan_range/2)
        end_angle = int(self.scan_angle+self.scan_range/2)
        # print(msg.ranges[start_angle:end_angle],'\r')
        self.points_r_theta = []
        self.points_x_y = []
        # print(start_angle)
        for ind, range_ in enumerate(msg.ranges[start_angle:end_angle]):
            # print("loop\r")
            current_angle = ind + start_angle
            # self.points_r_theta[ind, 0] = range_
            # self.points_r_theta[ind, 1] = current_angle

            if np.isfinite(range_):
                # print('finite\r')
                self.points_r_theta.append((range_, current_angle))

                x = range_ * np.cos((current_angle)*3.14/180.0)
                y = range_ * np.sin((current_angle)*3.14/180.0)

                self.points_x_y.append((x, y))
        # print(len(self.points_x_y),'\r')

                # self.points_r_theta[0, ind] = range_
                # self.points_r_theta[1, ind] = current_angle

                # x = range_ * np.cos((current_angle)*3.14/180.0)
                # y = range_ * np.sin((current_angle)*3.14/180.0)
                # # self.points_x_y[ind, 0] = x
                # # self.points_x_y[ind, 1] = y

                # self.points_x_y[0, ind] = x
                # self.points_x_y[1, ind] = y
            # print(self.points_x_y,'\r')

    def updateDesiredAngle(self):
        # get a best fit line through the points
        # returns True if angle updated successfully

        # Check that we have enough info to continue
        if not self.points_x_y or len(self.points_x_y) < 2:
            return False

        # print(self.points_x_y,'\r')
        num_points = len(self.points_x_y)

        # Convert xy list to numpy array
        # print(type(num_points),'\r')
        xs = np.zeros(num_points)
        ys = np.zeros(num_points)
        for index, point in enumerate(self.points_x_y):
            xs[index] = point[0]
            ys[index] = point[1]

        # print("xs", xs,'\r')
        # print("ys", ys, '\r')

        # xs = self.points_x_y[:, 0]
        # ys = self.points_x_y[:,1]
        xmin, xmax = min(xs), max(xs)
        pfit, stats = Polynomial.fit(xs, ys, 1, full=True, window=(xmin, xmax), domain=(xmin, xmax))
        _, slope = pfit
        # print("slope: ", slope, '\r')
        # print("tan slope: ", np.arctan(slope), '\r')
        # print("desired angle: ", np.arctan(slope), '\r')

        # Turn that into an angle
        self.desired_angle = np.arctan(slope) * 180.0/3.14
        # print(self.desired_angle,'\r')

        return True

    def run(self):
        r = rospy.Rate(10)
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run\r")
        while not rospy.is_shutdown():
            # print("hearbeat\r")
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

                # Constant linear speed
                twist_msg.linear.x = 0.3

                # Update the desired angle
                if self.updateDesiredAngle():
                    # print(len(self.points_x_y),'\r')
                    # Run Proportional Control.
                    # desired_angle = 0  or 360 if robot is properly aligned
                    print(self.desired_angle,'\r')
                    if self.desired_angle < 0:
                        twist_msg.angular.z = 0.05 * self.desired_angle
                    elif self.desired_angle > 0:
                        twist_msg.angular.z = 0.015 * self.desired_angle

                    # print(twist_msg.angular.z, '\r')
                    # if (self.desired_angle > 0.0 and self.desired_angle <= 180.0):
                    #     twist_msg.angular.z = 0.01 * self.desired_angle
                    # else:
                    #     twist_msg.angular.z = 0.01 * self.desired_angle
                    
                    # print("twist_msg.angular.z",twist_msg.angular.z,'\r')
                    # Enforce limits on angular velocity
                    if twist_msg.angular.z > self.max_angular_speed:
                        twist_msg.angular.z = self.max_angular_speed

                    elif twist_msg.angular.z < - self.max_angular_speed:
                        twist_msg.angular.z = - self.max_angular_speed


                # # Check the difference between the two points
                # # turn either left or right proportionally dependent on the difference
                # # fixed forward motion
                # if np.isinf(self.point_1): self.point_1 = 1000
                # if np.isinf(self.point_2): self.point_2 = 1000

                # print(repr(self.point_1), ' | ', repr(self.point_2),'\r')
                # twist_msg.linear.x = 0.3
                # twist_msg.angular.z = - (self.point_1 - self.point_2)

                # if twist_msg.angular.z > 1.0: twist_msg.angular.z = 1.0
                # elif twist_msg.angular.z < -1.0: twist_msg.angular.z = -1.0

                self.pub.publish(twist_msg)

            elif not self.active:
                self.pub.publish(Twist())

            r.sleep()
            # print("past sleeping\r")

        # print("rospy was shutdown")
        # Set terminal back to normal and stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        self.pub.publish(Twist())



if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
