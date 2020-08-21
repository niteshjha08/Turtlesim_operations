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
		print('init done')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
		self.rt_pose=rospy.Subscriber('/rt_real_pose',Pose,self.chase)
		self.rate=rospy.Rate(10)
		self.pose_sub=rospy.Subscriber('/turtle2/pose',Pose,self.callback)
		
	def callback(self,data):
		self.position=data

	def step_vel(self,v_final,v_initial,t_final,t_initial):
		
		K_accel=0.1
		K_decel=0.3
		if(v_final.linear.x-v_initial.linear.x>0):k=K_accel
		else: k=K_decel
		step_vel=Twist()
		
		step=k*(v_final.linear.x-v_initial.linear.x)

		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)
		#while(v_final.linear.x-v_initial.linear.x>step):
		#	print('in loop')
		#	v_initial.linear.x=v_initial.linear.x+step
		#	v_intial.angular.z=v_initial.angular.z+step
		#	self.vel_pub.publish(v_intial)
		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		
		self.vel_pub.publish(v_initial)
				

	def rotate(self,angle):
		vel=Twist()
		i=0
		while(abs((angle*pi/180)-(self.position.theta)))>0.05:
			vel.angular.z=(angle*pi/180)-(self.position.theta)
			self.vel_pub.publish(vel)
			

	def go_to_goal(self,goal):	
		start_time=time.time()
		vel=Twist()
		Kp=1
		Kd=0.1
		Ki=0.0001
		dist_prev=(self.position.x-goal.x)**2+(self.position.y-goal.y)**2
		e_sum=0
		vel_prev=Twist()
		t_initial=0
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum
			vel.angular.z=6*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
			
			t_final=time.time()
			
			self.step_vel(vel,vel_prev,t_final,t_initial)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist
			vel_prev.linear.x=vel.linear.x
			vel_prev.angular.z=vel_prev.angular.z
			
			
			if(dist<0.1):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",start_time-stop_time)
		
	def grid(self):
		goal=Pose()
		grid_corners=[(1,1,0),(10,1,90),(10,3,180),(1,3,90),(1,5,0),(10,5,90),(10,7,180),(1,7,90),(1,9,0),(10,9,0)]
		for i in range(len(grid_corners)):
			goal.x=grid_corners[i][0]
			goal.y=grid_corners[i][1]
			self.go_to_goal(goal)
			self.rotate(grid_corners[i][2])
			#time.sleep(1)
	def check_distance(self,rt_pose):
		dist=dist=sqrt((rt_pose.x-self.position.x)**2+(rt_pose.y-self.position.y)**2)
		print('current distance:',dist)
		if(dist<=3):
			print('chase completed')
			goal=Twist()
			self.vel_pub.publish(goal)
			
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


x=turtle()
rospy.spin()


		

