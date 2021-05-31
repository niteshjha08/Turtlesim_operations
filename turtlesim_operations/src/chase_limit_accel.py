#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import time

class turtle():
	def __init__(self):
		rospy.init_node('chase')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
		self.rt_pose=rospy.Subscriber('/rt_real_pose',Pose,self.chase)
		self.rate=rospy.Rate(10)
		self.pose_sub=rospy.Subscriber('/turtle2/pose',Pose,self.callback)
		
	def callback(self,data):
		self.position=data

	def step_vel(self,v_final,v_initial,time_sent):
		K_accel=0.1
		K_decel=0.3
		max_accel_linear=1500    #time gaps of calling this function are of order 10e-4. Hence accel becomes of order 1.5units/s^2.
		max_decel_linear=-1500
		max_accel_angular=6000
		max_decel_angular=-6000
		if(v_final.linear.x-v_initial.linear.x>0):  #Setting constants according to acceleration or deceleration situation
			k=K_accel
			max_steps_linear=max_accel_linear
			max_steps_angular=max_accel_angular
		else: 
			k=K_decel
			max_steps_linear=max_decel_linear
			max_steps_angular=max_decel_angular

		step_vel=Twist()
		step=k*(v_final.linear.x-v_initial.linear.x)		#Incrementing in steps to keep accel in limit
		t_now=time.time()
		dt=t_now-time_sent
		if(abs(step/dt)>abs(max_steps_linear)):
			step=dt*max_steps_linear			#set step corresponding to max accel
			print("MAX LINEAR ACCEL/DECEL REACHED!!!!")
			
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)		#Incrementing in steps to keep accel in limit. Can be faster than linear component
		if(abs(stepangular/dt)>abs(max_steps_angular)):
			stepangular=dt*max_steps_angular			#set step corresponding to max accel
			print("MAX ANGULAR ACCEL/DECEL REACHED!!!!")

		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		self.vel_pub.publish(v_initial)
		timer=time.time()
		return(v_initial,timer)				

	def go_to_goal(self,goal):	
		start_time=time.time()
		vel=Twist()
		Kp=1
		Kd=0.1
		Ki=0.0001
		dist_prev=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
		e_sum=0					#Initialise error summation terms for integral control
		vel_prev=Twist()
		t_final=time.time()
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum		#PID Implementation
			vel.angular.z=6*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)		#Angle needed to turn
			vel_prev,t_final=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist

			if(dist<0.1):
				stop_time=time.time()
				print("Reached goal")
				break
#This function is called on receiving rt_real_pose. It checks distance. If not in range, goes towards target.		
	def chase(self,target):
		
		dist=sqrt((self.position.x-target.x)**2+(self.position.y-target.y)**2)
		print("Current distance:",dist)
		if(dist<3):
			print("Distance less than 3. Caught RT\n")
			self.rt_pose.unregister()
			rospy.set_param('caughtStatus',True)
		else:
			print("Moving now")
			self.go_to_goal(target)
			

time.sleep(10)

x=turtle()
rospy.spin()

		

