#!/usr/bin/env python3

import rospy
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
Polynomial = np.polynomial.Polynomial
# from scipy.spatial.transform import Rotation as R

import tty

import select
import sys
import termios

# For visualizing lidar points
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
        rospy.Subscriber('/scan', LaserScan, self.callbackFunc)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_vis = rospy.Publisher('/wall_points', MarkerArray, queue_size=10)
        self.key = None

        self.start = False
        self.active = False

        # self.point_1 = None
        # self.point_2 = None

        self.scan_angle = 245 # 270 for right wall, 90 for left
        self.scan_range = 60

        self.max_angular_speed = 1.0

        # Make data containers easily accessible for later
        self.points_r_theta = None # np.zeros((2, self.scan_range))
        self.points_x_y = None # np.zeros((2, self.scan_range))
        # self.desired_angle = None # wall angle
        self.wall_slope = None

        self.front_range = None
        self.max_front_range = 1.0 # m

        self.state = self.followWallState()

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run.\r")

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def callbackFunc(self, msg):
        # Don't update if all lidar data is out of range
        if all(np.isinf(range_) for range_ in msg.ranges):
            return None

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
        
        if np.isinf(msg.ranges[0]):
            self.front_range = self.max_front_range + 0.01
        else:
            self.front_range = msg.ranges[0]

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
        # print('slope : ', slope,'\r')

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
        # Follow the wall
        print("Follow wall. ",self.front_range,"\r")

        # Switch states if we are going to crash
        if self.front_range and self.front_range < self.max_front_range:
            return self.turnLeftState()

        twist_msg = Twist()

        # Update the desired angle
        if self.updateDesiredAngle():
            twist_msg.linear.x = 0.3

            # Run proportional control based on slope
            twist_msg.angular.z = self.wall_slope

            # print(self.wall_slope, '|', twist_msg.angular.z,'\r')

            # Enforce limits on angular velocity
            if twist_msg.angular.z > self.max_angular_speed:
                twist_msg.angular.z = self.max_angular_speed

            elif twist_msg.angular.z < - self.max_angular_speed:
                twist_msg.angular.z = - self.max_angular_speed

            self.pub.publish(twist_msg)

        return self.followWallState
    
    def turnLeftState(self):
        print("Turn left. ",self.front_range,"\r")
        # Turn left until crash succesfully avoided
        if self.front_range and self.front_range < self.max_front_range:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = 0.7
            self.pub.publish(twist_msg)
            return self.turnLeftState
        else:
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

            # publish all the points of the lidar scan as markers
            if self.points_r_theta:
                marker_array = MarkerArray()
                for range_, theta in self.points_r_theta:
                    marker_array.markers.append(build_lidar_marker(theta, range_))
                self.pub_vis.publish(marker_array)
            r.sleep()

        # Set terminal back to normal and stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.pub.publish(Twist())

if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
