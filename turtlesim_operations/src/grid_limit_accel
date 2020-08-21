#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import time

class turtle():
	def __init__(self):
		rospy.init_node('goal_nav')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
		self.rate=rospy.Rate(20)   			#Set at 20Hz
		self.pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.callback)
		
	def callback(self,data):
		self.position=data

	def step_vel(self,v_final,v_initial,time_sent):
		K_accel=0.3
		K_decel=0.3
		max_accel_linear=12000   #time gaps of calling this function are of order 10e-4.
		max_decel_linear=-12000
		max_accel_angular=12000
		max_decel_angular=-12000
	
		if(v_final.linear.x-v_initial.linear.x>0):  #Setting constants according to acceleration or deceleration situation
			k=K_accel
			max_steps_linear=max_accel_linear
			max_steps_angular=max_accel_angular
		else: 
			k=K_decel
			max_steps_linear=max_decel_linear
			max_steps_angular=max_decel_angular

		step_vel=Twist()
		step=k*(v_final.linear.x-v_initial.linear.x)
		t_now=time.time()
		dt=t_now-time_sent
		if(abs(step/dt)>abs(max_steps_linear)):
			step=dt*max_steps_linear
			print(step)
			print("MAX_ACCEL/DECEL_LINEAR REACHED!")
			
		
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)
		if(abs(stepangular/dt)>abs(max_steps_angular)):
			stepangular=dt*max_steps_angular
			print(stepangular)
			print("MAX ANGULAR ACCEL/DECEL REACHED!!!!")
	
		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		self.vel_pub.publish(v_initial)
		timer=time.time()
		return(v_initial,timer)				
#This function takes in angle inputs and imparts rotation to turtle using proportional control
	def rotate(self,angle):
		vel=Twist()
		i=0
		while(abs((angle*pi/180)-(self.position.theta)))>0.05:
			vel.angular.z=3*((angle*pi/180)-(self.position.theta))
			self.vel_pub.publish(vel)
			

	def go_to_goal(self,goal):	
		start_time=time.time()
		vel=Twist()
		Kp=1
		Kd=0.2
		Ki=0.0003
		dist_prev=(self.position.x-goal.x)**2+(self.position.y-goal.y)**2
		e_sum=0										#Initialise error summation terms for integral control
		vel_prev=Twist()
		t_final=time.time()
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum  				#PID
			vel.angular.z=4*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
			vel_prev,t_final=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist			
			if(dist<0.1):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",stop_time-start_time)
#This function drives the grid action. Array consists of corner coordinates and angle to rotate at the end ot traversal.	
	def grid(self):
		goal=Pose()
		grid_corners=[(1,1,0),(10,1,90),(10,3,180),(1,3,90),(1,5,0),(10,5,90),(10,7,180),(1,7,90),(1,9,0),(10,9,0)]
		for i in range(len(grid_corners)):
			goal.x=grid_corners[i][0]
			goal.y=grid_corners[i][1]
			self.go_to_goal(goal)				#Go to corner point
			self.rotate(grid_corners[i][2])			#Rotate by assigned degrees
			
			


x=turtle()
x.grid()

		

