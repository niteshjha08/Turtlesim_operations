#! /usr/bin/env python

import rospy
from math import sqrt
from turtlesim.msg import Pose

def check_dist(rt_pose,pt_pose):
	dist=sqrt((rt_pose.linear.x-pt_pose.linear.x)**2+(rt_pose.linear.y-pt_pose.linear.y)**2)
	if(dist<=10):
		print("Police Turtle has caught the Robber Turtle! Task Ended")

rospy.init_node('distance_checker')
pt_sub=rospy.Subscriber('/turtle2/pose',Pose,
