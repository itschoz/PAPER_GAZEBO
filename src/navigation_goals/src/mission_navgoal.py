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

def create_nav_goal(x, y, yaw):
    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
    Returns a MoveBaseGoal"""
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = '/map' # Note: the frame_id must be map
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)
    
    # Orientation of the robot is expressed in the yaw value of euler angles
    #angle = radians(yaw) # angles are expressed in radians
    angle = 1
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return mb_goal

def callback_pose(data):
    """Callback for the topic subscriber.
       Prints the current received data on the topic."""
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    rospy.loginfo("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)) + "º")

if __name__=='__main__':
    rospy.init_node("navigation_snippet")
    
    # Read the current pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
    
    # Connect to the navigation action server
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Connecting to /move_base AS...")
    nav_as.wait_for_server()
    rospy.loginfo("Connected.")
    
    #Mission 1.1
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(5.83, 4.016, 0.0)
    rospy.loginfo("Sending goal to x=-5.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()


#CURVE
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.296, 3.548, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.595, 3.035, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()



    #Mission 2.1
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.725, 2.4489, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()



    #Mission 3.1
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.684, -0.245, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 3.2
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.551, -2.719, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 3.3
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.374, -3.242, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 3.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(6.00487, -3.72836, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 3.5
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(5.402, -4.038, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()



    #Mission 4.1
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(4.568, -4.099, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.2
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-1.487, -4.048, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.3
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-5.997, -4.03, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-6.819, -3.619, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.334, -3.141, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.667, -2.533, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.776, -1.933, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.779, 0.416, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.9, 2.55, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.82, 3.16, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-7.49, 3.7, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-6.87, 4.088, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()


if attack = True:
    #Mission 4.4
    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(-6.5, 3.16, 0.0)
    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()

    #Mission 4.4
 #   rospy.loginfo("Creating navigation goal...")
 #   nav_goal = create_nav_goal(-1, 4, 0.0)
 #   rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
 #   nav_as.send_goal(nav_goal)
 #   rospy.loginfo("Waiting for result...")
 #   nav_as.wait_for_result()

    #Mission 4.4
#    rospy.loginfo("Creating navigation goal...")
#    nav_goal = create_nav_goal(5.138, 4.34, 0.0)
#    rospy.loginfo("Sending goal to x=-9.0 y=5.0 yaw=0...")
#    nav_as.send_goal(nav_goal)
#    rospy.loginfo("Waiting for result...")
#    nav_as.wait_for_result()



    nav_res = nav_as.get_result()
    nav_state = nav_as.get_state()
    rospy.loginfo("Done!")
    print "Result: ", str(nav_res) # always empty, be careful
    print "Nav state: ", str(nav_state) # use this, 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
