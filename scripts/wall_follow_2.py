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

        self.scan_angle = 250 # 270 for right wall, 90 for left
        self.scan_range = 60

        self.max_angular_speed = 1.0

        # Make data containers easily accessible for later
        self.points_r_theta = None # np.zeros((2, self.scan_range))
        self.points_x_y = None # np.zeros((2, self.scan_range))
        # self.desired_angle = None # wall angle
        self.wall_slope = None

        self.state = self.followWallState()

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run.\r")

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callbackFunc(self, msg):
        # update point data
        start_angle = int(self.scan_angle)
        end_angle = int(self.scan_angle+self.scan_range)
        self.points_r_theta = []
        self.points_x_y = []
        for ind, range_ in enumerate(msg.ranges[start_angle:end_angle]):
            current_angle = ind + start_angle

            if np.isfinite(range_):
                self.points_r_theta.append((range_, current_angle))

                x = range_ * np.cos((current_angle)*3.14/180.0)
                y = range_ * np.sin((current_angle)*3.14/180.0)

                self.points_x_y.append((x, y))


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

        xmin, xmax = min(xs), max(xs)
        pfit, stats = Polynomial.fit(xs, ys, 1, full=True, window=(xmin, xmax), domain=(xmin, xmax))
        _, slope = pfit
        # print("slope: ", slope, '\r')
        # print("tan slope: ", np.arctan(slope), '\r')
        # print("desired angle: ", np.arctan(slope), '\r')

        # Turn that into an angle
        print('slope : ', slope,'\r')

        self.wall_slope = slope

        return True
    
    def updateUserInput(self):
        self.key = None
        if self.keyWasPressed():
            self.key = sys.stdin.read(1)
        if self.key == '\x03': print ("Shutting down node\r"); return "SHUTDOWN"
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
        return None

    def isRunning(self):
        return self.start and self.active
    
    def followWallState(self):
        twist_msg = Twist()

        # Update the desired angle
        if self.updateDesiredAngle():
            twist_msg.linear.x = 0.3

            # Run proportional control based on slope
            twist_msg.angular.z = self.wall_slope
            
            print(self.wall_slope, '|', twist_msg.angular.z,'\r')

            # Enforce limits on angular velocity
            if twist_msg.angular.z > self.max_angular_speed:
                twist_msg.angular.z = self.max_angular_speed

            elif twist_msg.angular.z < - self.max_angular_speed:
                twist_msg.angular.z = - self.max_angular_speed

            self.pub.publish(twist_msg)
        
        return self.followWallState
        

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.updateUserInput() == "SHUTDOWN":
                break
            elif self.isRunning():
                self.state = self.state()
            else:
                self.pub.publish(Twist())
            r.sleep()

        # Set terminal back to normal and stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.pub.publish(Twist())



if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
