#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import numpy as np
import time

class turtle():
	def __init__(self):
		rospy.init_node('circle')
		print('init done')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
		self.pose_pub=rospy.Publisher('/rt_real_pose',Pose,queue_size=10)
		self.noisy_pose_pub=rospy.Publisher('/rt_noisy_pose',Pose,queue_size=10)
		self.rate=rospy.Rate(10)		
		self.pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.callback)
		
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
			
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)		#Incrementing in steps to keep accel in limit
		if(abs(stepangular/dt)>abs(max_steps_angular)):		
			stepangular=dt*max_steps_angular			#set step corresponding to max accel
			
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
		dist_prev=(self.position.x-goal.x)**2+(self.position.y-goal.y)**2
		e_sum=0						#Initialise error summation terms for integral control
		vel_prev=Twist()
		t_initial=0
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum
			vel.angular.z=6*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
			
			t_final=time.time()
			
			vel_prev=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist

			if(dist<0.1):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",start_time-stop_time)
#This function takes a pose and adds gaussian noise to all components of it. normal(mu,sigma) are parameters that can be adjusted.
	def gaussian_noise(self):
		noisy_pose=Pose()
		noisy_pose.x=self.position.x+np.random.normal(0,2)
		noisy_pose.y=self.position.y+np.random.normal(0,2)
		noisy_pose.theta=self.position.theta+np.random.normal(0,0.2)
		noisy_pose.linear_velocity=self.position.linear_velocity+np.random.normal(0,0.2)
		noisy_pose.angular_velocity=self.position.angular_velocity+np.random.normal(0,0.2)
		self.noisy_pose_pub.publish(noisy_pose)
#Draws circles from defined radius and velocity. 				
	def circles(self,vel,radius):
		vel_msg=Twist()
		vel_msg.linear.x=vel
		vel_msg.angular.z=vel_msg.linear.x/radius     		   # follow equation v=rw
		vel_prev=Twist()
		t_initial=time.time()
		zero_vel=Twist()
		while(not rospy.is_shutdown()):
			if (rospy.get_param('caughtStatus')):  		#Comment out this line if not running chase launch files. This checks if RT has been caught yet, and only runs until then
				self.vel_pub.publish(zero_vel)              
				print("Im CAUGHT!!")
				break
			else:
				t_final=time.time()
				vel_prev,t_final=self.step_vel(vel_msg,vel_prev,t_final)		#gradually increase velocity to trace circles. Hence spiral can be seen initially.
				self.rate.sleep()
				if(time.time()-t_initial>5):
					self.pose_pub.publish(self.position)
					t_initial=time.time()
					self.gaussian_noise()
				
			


x=turtle()

v=2   # Set the velocity as required
r=2   # Set radius. Angular velocity is calculated using v=rw
x.circles(v,r)

		

