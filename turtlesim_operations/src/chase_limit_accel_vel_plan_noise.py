#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi, sin, cos, degrees, radians, ceil
import time
import numpy as np
i=0
a=[]
class turtle():
	def __init__(self):
		rospy.init_node('chase')
		self.position=Pose()
		self.v_max=1				#Setting v_max
		self.vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
		self.rt_pose=rospy.Subscriber('/rt_real_pose',Pose,self.actual_gap)
		self.rt_noise_pose=rospy.Subscriber('/rt_noisy_pose',Pose,self.plan)
		self.rate=rospy.Rate(8)
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
		
		step=k*(v_final.linear.x-v_initial.linear.x)				#Incrementing in steps to keep accel in limit
		t_now=time.time()
		dt=t_now-time_sent
		if(abs(step/dt)>abs(max_steps_linear)):
			
			step=dt*max_steps_linear				#set step corresponding to max accel
			
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)			#Incrementing in steps to keep accel in limit
		if(abs(stepangular/dt)>abs(max_steps_angular)):
			stepangular=dt*max_steps_angular			#set step corresponding to max accel
			
		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		if(v_initial.linear.x>self.v_max):
			v_initial.linear.x=self.v_max
			
		self.vel_pub.publish(v_initial)
		timer=time.time()
		
		return(v_initial,timer)				

#This finds distance between self and a tuple	
	def distance(self,point):
		return(sqrt((self.position.x-point[0])**2+(self.position.y-point[1])**2))

#This finds distance between self and goal pose	
	def euclidean_distance(self,goal):
		return(sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2))	


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
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum			#PID Implementation
			vel.angular.z=6*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
			vel_prev,t_final=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist

			if(dist<0.1):
				stop_time=time.time()
				print("Reached goal")
				break
		print("Total time taken:",start_time-stop_time)
		
#This function plans the future points, and moves to them.	
	def plan(self,target):
		print("Current distance:",self.euclidean_distance(target))
		predicted_pos=Pose()
		if(np.shape(a)[0]<3):
			a.append([target.x,target.y])		
		print(a)
		if(np.shape(a)[0]>=3):			#Need minimum of 3 points to define circle	
			center,radius=self.define_circle(a[0],a[1],a[2])
			next_point=self.next_point(a[1],a[2],center,radius)		#Get position of RT in next 5 seconds 
			a.append(next_point)
			#print("next_point:",next_point)
			plan_length=1
			while(True):		#Loop until reachable point is found
				print("Planning for {0} s ahead:".format(plan_length*5))
				if(self.distance(next_point)/self.v_max>(5*plan_length)):			#check if target is reachable in timeslot
					next_point=self.next_point(a[-2],a[-1],center,radius)
					a.append(next_point)
					print("Estimated time:",(self.distance(next_point)/self.v_max))
					plan_length=plan_length+1
					
				else:
					predicted_pos.x=next_point[0]
					predicted_pos.y=next_point[1]
					print("Planned for pos:",predicted_pos)
					self.go_to_goal(predicted_pos)
					break
					
#This function takes 3 points and finds circle passing all of them, then returning its radius and center	
	def define_circle(self,p1, p2, p3):
	    temp = p2[0] * p2[0] + p2[1] * p2[1]
	    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
	    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
	    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

	    if abs(det) < 1.0e-6:
		return (None, np.inf)

	    # Center of circle
	    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
	    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

	    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
	    return ((cx, cy), radius)
#Calculate next point in series using previous two points	
	def next_point(self,p2,p3,center,radius):
		theta2=atan2(p2[1]-center[1],p2[0]-center[0])
		theta3=atan2(p3[1]-center[1],p3[0]-center[0])
		dtheta=degrees(theta2-theta3)
		theta4=degrees(theta3)-dtheta
		nextval=([0,0])
		if(theta4<0):
			theta4=theta4+360
		nextval[0]=center[0]+radius*cos(radians(theta4))
		nextval[1]=center[1]+radius*sin(radians(theta4))
		return (nextval)

#This function recieves true position and keeps checking if gap is close enough to end chase
	def actual_gap(self,pos):
		if(self.euclidean_distance(pos)<3):
			print('My Position:',self.position.x,self.position.y)
			print('Target Position:',pos.x,pos.y)
			print("Distance less than 3. Caught RT\n")
			self.rt_pose.unregister()
			rospy.set_param('caughtStatus',True)

		
			

time.sleep(10)
x=turtle()
rospy.spin()

		

