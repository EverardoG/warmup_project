#!/usr/bin/env python3

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8MultiArray

import random
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
        rospy.Subscriber('/scan', LaserScan, self.scanCallbackFunc)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.key = None

        self.start = False
        self.active = False

        self.scan_range = 60

        self.max_angular_speed = 1.0

        # Make data containers easily accessible for later
        self.points_r_theta = None # np.zeros((2, self.scan_range))
        self.points_x_y = None # np.zeros((2, self.scan_range))
        
        self.wall_slope = None
        self.turn_direction = None

        self.clustser_dist = None
        self.min_cluster_dist = 1.0 # m

        self.state = self.followHumanState()

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run.\r")

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def scanCallbackFunc(self, msg):
        # Don't update if all lidar data is out of range
        if all(np.isinf(range_) for range_ in msg.ranges):
            self.points_r_theta = None
            self.points_x_y = None
            return None

        # Setup data containers
        self.points_r_theta = []
        self.points_x_y = []

        # update point data
        upper_angle = int(self.scan_range/2)
        for angle, range_ in enumerate(msg.ranges[0:upper_angle]):
            if np.isfinite(range_):
                self.points_r_theta.append((range_, angle))

                x = range_ * np.cos((angle)*3.14/180.0)
                y = range_ * np.sin((angle)*3.14/180.0)

                self.points_x_y.append((x, y))

        lower_angle = int(360-self.scan_range/2)
        for ind, range_ in enumerate(msg.ranges[lower_angle:360]):
            angle = 360 - self.scan_range/2
            if np.isfinite(range_):
                self.points_r_theta.append((range_, angle))

                x = range_ * np.cos((angle)*3.14/180.0)
                y = range_ * np.sin((angle)*3.14/180.0)

                self.points_x_y.append((x, y))

    def updateDesiredAngle(self):
        # Calculate the center of the cluster of points
        # Determine the angle to the center
        # Return false if there are no points

        # Make sure the points are valid
        if not self.points_x_y or len(self.points_x_y) < 1:
            return False

        # Organize points into numpy arrays
        num_points = len(self.points_x_y)
        xs = np.zeros(num_points)
        ys = np.zeros(num_points)
        for index, point in enumerate(self.points_x_y):
            xs[index] = point[0]
            ys[index] = point[1]

        # Find the center of all points
        center_x = np.mean(xs)
        center_y = np.mean(ys)

        # Find the slope to that point
        # NOTE: The lidar is at 0,0
        slope = center_y/center_x

        # Find the distance to the center
        self.obj_range = np.linalg.norm([center_x, center_y])
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
            print("Robot is starting human follow mode.\r")
        # toggle whether robot is active
        if self.start and self.key == " ":
            self.active = not self.active
            if self.active: print("Continuing Robot.\r")
            else: print("Pausing Robot.\r")
        return None

    def isRunning(self):
        return self.start and self.active
    
    def followHumanState(self):
        print("Follow human\r")
        # Follow the wall
        # print("Follow wall. ",self.front_range,"\r")

        twist_msg = Twist()

        # Update the desired angle
        if self.updateDesiredAngle():
            # twist_msg.linear.x = 0.3

            # Run linear control based on distance
            # Faster when further away, slower when closer
            if self.obj_range < 0.5:
                twist_msg.linear.x = 0.0

            else:
                twist_msg.linear.x = 2.0/3.0 * (self.obj_range-0.5)
                if twist_msg.linear.x > 2.0:
                    twist_msg.linear.x = 2.0

            # Run proportional control based on slope
            twist_msg.angular.z = 2 * self.wall_slope

            # print(self.wall_slope, '|', twist_msg.angular.z,'\r')

            # Enforce limits on angular velocity
            if np.abs(twist_msg.angular.z) < 0.1:
                twist_msg.angular.z = 0.0
            elif twist_msg.angular.z > self.max_angular_speed:
                twist_msg.angular.z = self.max_angular_speed

            elif twist_msg.angular.z < - self.max_angular_speed:
                twist_msg.angular.z = - self.max_angular_speed

            self.pub.publish(twist_msg)

            return self.followHumanState
        
        # Look for a human if robot doesn't see anything
        return self.searchForHumanState
    
    def searchForHumanState(self):
        print("Search for human\r")
        # Check if you've found a human
        if self.points_x_y and len(self.points_x_y) > 1:
            return self.followHumanState()

        # Turn towards the last seen location of the human
        if self.wall_slope:
            self.turn_direction = np.sign(self.wall_slope)
        # Or pick randomly if you never saw anything
        elif not self.turn_direction:
            self.turn_direction = random.choice([-1,1])
        
        twist_msg = Twist()
        twist_msg.angular.z = self.turn_direction * 0.7
        self.pub.publish(twist_msg)

        return self.searchForHumanState
        

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
