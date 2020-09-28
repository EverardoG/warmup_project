#!/usr/bin/env python3

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8MultiArray
from visualization_msgs.msg import MarkerArray, Marker

import random
import numpy as np
Polynomial = np.polynomial.Polynomial
# from scipy.spatial.transform import Rotation as R

import tty

import select
import sys
import termios

def build_lidar_marker(angle, range_):
    marker = Marker()
    marker.header.frame_id = "laser_link";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "lidar_visualization";
    marker.id = angle;
    marker.type = Marker.SPHERE;
    marker.action = Marker.ADD;
    marker.pose.position.x = range_ * np.cos((angle+180)*3.14/180.0);
    marker.pose.position.y = range_ * np.sin((angle+180)*3.14/180.0);
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1
    marker.scale.z = 0.1;
    marker.color.a = 0.9; # Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker

# 🐛 The Worm is Watching
class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.scanCallbackFunc)
        rospy.Subscriber('/bump', Int8MultiArray, self.bumpCallbackFunc)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_vis = rospy.Publisher('/visualize_person', MarkerArray, queue_size=10)
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
        self.bump = False

        self.clustser_dist = None
        self.min_cluster_dist = 1.0 # m

        self.state = self.followHumanState()

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run.\r")

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def bumpCallbackFunc(self, msg):
        # Update bump info
        if any((int_ == 1 for int_ in msg.data)):
            self.bump = True
        else:
            self.bump = False

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
            angle = ind - self.scan_range/2
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
    
    def victoryState(self):
        print("Defeated human\r")
        if not self.updateDesiredAngle():
            return self.searchForHumanState
        twist_msg = Twist()
        if self.obj_range > 1:
            return self.followHumanState

        self.pub.publish(twist_msg)
        return self.victoryState
    
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
            # publish all the points of the lidar scan as markers
            if self.points_r_theta:
                marker_array = MarkerArray()
                for range_, theta in self.points_r_theta:
                    # print(theta,'\r')
                    marker_array.markers.append(build_lidar_marker(int(theta), range_))
                self.pub_vis.publish(marker_array)
            r.sleep()

        # Set terminal back to normal and stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.pub.publish(Twist())

if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
