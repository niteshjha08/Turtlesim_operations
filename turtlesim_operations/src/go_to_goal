#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt
import time
class turtle():
	def __init__(self):
		rospy.init_node('goal_nav')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
		self.rate=rospy.Rate(10)                         # set at 10 Hz
		self.pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.callback)
		
	def callback(self,data):                                 #Update value of poses
		self.position=data
	
	def go_to_goal(self):	
		goal=Pose()
		goal.x=input("Enter x coordinate of goal:")
		goal.y=input("Enter y coordinate of goal:")
		start_time=time.time()
		vel=Twist()
		Kp=1
		Kd=0.1
		Ki=0.0001
		dist_prev=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
		e_sum=0           										#Initialise error summation terms for integral control
		steer_sum=0
		steer_prev=(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum							#PID Implementation
			steer=(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)   #Angle needed to turn
			dsteer=steer-steer_prev
			vel.angular.z=6*Kp*steer+Kd*dsteer+Ki*steer_sum						#PID implementation
			self.vel_pub.publish(vel)
			self.rate.sleep()
			dist_prev=dist
			steer_prev=steer
			steer_sum=steer_sum+steer_prev
			if(dist<0.5):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",stop_time-start_time)
		




x=turtle()
while not rospy.is_shutdown():
	x.go_to_goal()

		

