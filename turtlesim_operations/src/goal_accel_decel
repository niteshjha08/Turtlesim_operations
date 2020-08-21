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
		self.rate=rospy.Rate(10)
		self.pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.callback)
		
	def callback(self,data):
		self.position=data

	def step_vel(self,v_final,v_initial,time_sent):
		K_accel=0.1
		K_decel=0.3
		max_accel_linear=1500    #time gaps of calling this function are of order 10e-4. Hence accel becomes of order 0.15units/s^2.
		max_decel_linear=-1500
		max_accel_angular=6000		#Anglular accel become 0.6deg/s^2
		max_decel_angular=-6000
		if(v_final.linear.x-v_initial.linear.x>0):
			
			k=K_accel
			max_steps_linear=max_accel_linear
			max_steps_angular=max_accel_angular
		else: 
			
			k=K_decel
			max_steps_linear=max_decel_linear
			max_steps_angular=max_decel_angular
		step_vel=Twist()
		step=k*(v_final.linear.x-v_initial.linear.x)    #Incrementing in steps to keep accel in limit
		t_now=time.time()
		dt=t_now-time_sent
		if(abs(step/dt)>abs(max_steps_linear)):
			print("MAX_ACCEL/DECEL_LINEAR REACHED!")
			step=dt*max_steps_linear				#set step corresponding to max accel
		
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)   	#Incrementing in steps to keep accel in limit. Can be faster than linear component
		if(abs(stepangular/dt)>abs(max_steps_angular)):
			print("MAX_ACCEL/DECEL_ANGULAR REACHED!")
			stepangular=dt*max_steps_angular			#set step corresponding to max accel
			
		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		self.vel_pub.publish(v_initial)
		timer=time.time()
		return(v_initial,timer)		



	def go_to_goal(self):	
		goal=Pose()
		goal.x=input("Enter x coordinate of goal:")
		goal.y=input("Enter y coordinate of goal:")
		start_time=time.time()
		vel=Twist()
		Kp=1
		Kd=0.1
		Ki=0.0001
		dist_prev=(self.position.x-goal.x)*(self.position.x-goal.x)+(self.position.y-goal.y)*(self.position.y-goal.y)
		e_sum=0														#Initialise error summation terms for integral control
		vel_prev=Twist()
		t_final=time.time()
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum									#PID Implementation
			vel.angular.z=4*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)	#Angle needed to turn
			vel_prev,t_final=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final	
			self.rate.sleep()
			dist_prev=dist
			
			if(dist<0.5):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",stop_time-start_time)
		




x=turtle()
x.go_to_goal()

		

