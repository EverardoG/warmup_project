#!/usr/bin/env python3

"""
Created on 20 September 2020
@author: Everardo Gonzalez
"""

import rospy
from visualization_msgs.msg import Marker

class MarkerNode(object):
    def __init__(self):
        rospy.init_node('marker_node')
        self.pub = rospy.Publisher('/markers', Marker, queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = "wheel_left_link";
        self.marker.header.stamp = rospy.Time.now();
        self.marker.ns = "my_namespace";
        self.marker.id = 0;
        self.marker.type = Marker.SPHERE;
        self.marker.action = Marker.ADD;
        self.marker.pose.position.x = 1;
        self.marker.pose.position.y = 2;
        self.marker.pose.position.z = 0.5;
        self.marker.pose.orientation.x = 0.0;
        self.marker.pose.orientation.y = 0.0;
        self.marker.pose.orientation.z = 0.0;
        self.marker.pose.orientation.w = 1.0;
        self.marker.scale.x = 1;
        self.marker.scale.y = 1
        self.marker.scale.z = 1;
        self.marker.color.a = 0.9; # Don't forget to set the alpha!
        self.marker.color.r = 0.0;
        self.marker.color.g = 0.0;
        self.marker.color.b = 1.0;

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.marker)
            r.sleep()

if __name__ == '__main__':
    marker_node = MarkerNode()
    marker_node.run()
