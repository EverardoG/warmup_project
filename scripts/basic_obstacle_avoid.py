#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
# Polynomial = np.polynomial.Polynomial
from scipy.spatial.transform import Rotation as R

import tty

import select
import sys
import termios

def build_lidar_marker(point_x, point_y, id_):
    marker = Marker()
    marker.header.frame_id = "laser_link";
    marker.header.stamp = rospy.Time.now();
    marker.ns = "lidar_visualization";
    marker.id = id_;
    marker.type = Marker.SPHERE;
    marker.action = Marker.ADD;
    marker.pose.position.x = point_x;
    marker.pose.position.y = point_y;
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

# Check if there are any points directly in front of the robot

# Two states
# Drive forward
#   Drive forward unless there is an object in the Neato's path
# Obstacle avoid
#   Turn left until the object in front of the Neato is to the right of the Neato
#   Then move forward until the path to the right of the Neato is clear

# 🐛 The Worm is Watching
class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('obstacle_avoid')
        rospy.Subscriber('/scan', LaserScan, self.scanCallbackFunc)
        rospy.Subscriber('/odom', Odometry, self.odomCallbackFunc)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_vis = rospy.Publisher('/lidar_visualization', MarkerArray, queue_size=10)

        self.key = None
        self.start = False
        self.active = False

        self.points_r_theta = None
        self.points_x_y = None

        self.obstacleIsAhead = False
        self.rightSideClear = False
        self.current_pos = np.array([None, None]) # [x, y]

        self.marker_array = None

        self.init_angle = None
        self.current_angle = None

        self.state = self.driveToGoalState

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print("Initialized node.\r")
        print("Hit ENTER to begin the run.\r")

    def keyWasPressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def odomCallbackFunc(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y

        r = R.from_quat([0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        euler_angles = r.as_euler('xyz', degrees=True)
        self.current_angle = euler_angles[2] + 180.0

    def scanCallbackFunc(self, msg):
        # Don't update if all lidar data is out of range
        if all(np.isinf(range_) for range_ in msg.ranges):
            return None

        self.points_r_theta = []
        self.points_x_y = []
        for angle, range_ in enumerate(msg.ranges[:360]):
            if np.isfinite(range_):
                self.points_r_theta.append((range_, angle))

                x = range_ * np.cos((angle + 180.0)*3.14/180.0)
                y = range_ * np.sin((angle + 180.0)*3.14/180.0)

                self.points_x_y.append((x, y))

        # save all the points of the lidar scan as markers
        self.marker_array = MarkerArray()
        for index, (point_x, point_y) in enumerate(self.points_x_y):
            self.marker_array.markers.append(build_lidar_marker(point_x, point_y, index))

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
    
    def driveToGoalState(self):
        print('driveToGoalState\r')
        if self.points_x_y:
            # Check if obstacle ahead of bot
            for point in self.points_x_y:
                if point[0] > -0.5:
                    if point[1] > -0.2 and point[1] < 0.2:
                        print(point,'\r')
                        self.init_angle = self.current_angle
                        return self.turnLeftState
            
            # Drive forward
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            self.pub.publish(twist_msg)
        
        return self.driveToGoalState
    
    def turnLeftState(self):
        print('turnLeftState\r')
        twist_msg = Twist()
        # Handle edge case where robot turns from 180 to -180
        if self.current_angle < self.init_angle and (self.current_angle - self.init_angle) < -0.01:
            angle_travelled = (180 - self.init_angle) + (180 + self.current_angle)
        else:
            angle_travelled = self.current_angle - self.init_angle

        # Run proportional control
        if angle_travelled < 90.0:
            twist_msg.angular.z = 0.008 * (90.0 - angle_travelled) + 0.01
            if twist_msg.angular.z > 1.0:
                twist_msg.angular.z = 1.0
            self.pub.publish(twist_msg)
            return self.turnLeftState
        else:
            return self.driveUntilClearState

    def driveUntilClearState(self):
        print('driveUntilClearState\r')
        if self.points_x_y:
            # Turn right once the closest point is 0.5 m away
            all_ranges = [pair[0] for pair in self.points_r_theta]
            closest_range = min(all_ranges)
            if closest_range > 0.5:
                self.init_angle = self.current_angle
                return self.turnRightState

        else:
            # Just turn right if there are no longer any points
            return self.turnRightState

        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        self.pub.publish(twist_msg)

        return self.driveUntilClearState
    
    def turnRightState(self):
        print('turnRightState\r')
        # Handle edge case where robot turns from 180 to -180
        if self.current_angle and self.init_angle:
            print(self.current_angle, " | ", self.init_angle, '\r')
            
            if self.init_angle < 180.0 and self.current_angle > 180:
                print("special case\r")
                angle_travelled = 360.0 + self.init_angle - self.current_angle

            else:
                print("normal case\r")
                angle_travelled = self.init_angle - self.current_angle
        
        else:
            angle_travelled = 0
            print("cant find angle information\r")

        print(angle_travelled,'\r')

        # Run proportional control
        if angle_travelled < 90.0:
            twist_msg = Twist()
            twist_msg.angular.z = - 0.008 * (90.0 - angle_travelled) - 0.01
            if twist_msg.angular.z < -1.0:
                twist_msg.angular.z = -1.0
            self.pub.publish(twist_msg)
            return self.turnRightState
        else:
            return self.driveToGoalState

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.updateUserInput() == "SHUTDOWN":
                break
            elif self.isRunning():
                self.state = self.state()
            else:
                self.pub.publish(Twist())

            self.pub_vis.publish(self.marker_array)
            r.sleep()

        # Set terminal back to normal and stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.pub.publish(Twist())

if __name__ == '__main__':
    wallfollow = WallFollowNode()
    wallfollow.run()
