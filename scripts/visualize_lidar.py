#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

import numpy as np

import rospy
from visualization_msgs.msg import MarkerArray, Marker

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

class VisualizeLidarNode(object):
    def __init__(self):
        rospy.init_node('visualize_lidar')
        rospy.Subscriber('/scan', LaserScan, self.callback_func)
        self.pub = rospy.Publisher('/marker', MarkerArray, queue_size=360)

    def callback_func(self, msg):
        # publish all the points of the lidar scan as markers
        marker_array = MarkerArray()
        for angle, range_ in enumerate(msg.ranges[:360]):
            if not np.isinf(range_):
                marker_array.markers.append(build_lidar_marker(angle, range_))
        self.pub.publish(marker_array)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    visualize_lidar = VisualizeLidarNode()
    visualize_lidar.run()
