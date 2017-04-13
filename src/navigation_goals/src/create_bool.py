#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 10 11:16:00 2014
@author: Sam Pfeiffer
Snippet of code on how to send a navigation goal and how to get the current robot position in map
Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal
Actual robot pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
import time
from std_msgs.msg import Int64


if __name__=='__main__':
    rospy.init_node("navigation_snippet1111111111")
    attack = 0
    pub = rospy.Publisher('detector', Int64,queue_size = 10)
    i = 0
    rate = rospy.Rate(10)
    while i < 600:
        if i == 400:
        	attack = 1
        	pub.publish(attack)
		i = 600
        i = i + 1
        rate.sleep()

