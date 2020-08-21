#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import time
i=0
class turtle():
	def __init__(self):
		rospy.init_node('chase')
		print('init done')
		self.position=Pose()
		self.vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
		self.rt_pose=rospy.Subscriber('/rt_real_pose',Pose,self.plan)
		self.rate=rospy.Rate(8)
		self.pose_sub=rospy.Subscriber('/turtle2/pose',Pose,self.callback)
		
	def callback(self,data):
		self.position=data

	def step_vel(self,v_final,v_initial,time_sent):
		global i
		print(i)
		i=i+1
		K_accel=0.1
		K_decel=0.3
		max_accel_linear=1500    #time gaps of calling this function are of order 10e-4. Hence accel becomes of order 1.5units/s^2.
		max_decel_linear=-1500
		max_accel_angular=6000
		max_decel_angular=-6000
		v_max=1
		print(v_final.linear)
		print('v_initial is this:',v_initial)
		print("End of v_intial")
		if(v_final.linear.x-v_initial.linear.x>0):  #Setting constants according to acceleration or deceleration situation
			k=K_accel
			max_steps_linear=max_accel_linear
			max_steps_angular=max_accel_angular
		else: 
			k=K_decel
			max_steps_linear=max_decel_linear
			max_steps_angular=max_decel_angular

		step_vel=Twist()
		print("v_final:",v_final.linear.x)
		print("v_initial:",v_initial.linear.x)
		
		step=k*(v_final.linear.x-v_initial.linear.x)
		t_now=time.time()
		dt=t_now-time_sent
		if(abs(step/dt)>abs(max_steps_linear)):
			print('step:',step,'dt:',dt,'curr_accel:',step/dt)
			step=dt*max_steps_linear
			print("MAX LINEAR ACCEL/DECEL REACHED!!!!")
			print("new accel:",step/dt)
			
		
		stepangular=4*k*(v_final.angular.z-v_initial.angular.z)
		if(abs(stepangular/dt)>abs(max_steps_angular)):
			print('stepangular:',step,'dt:',dt,'curr_accel_angular:',stepangular/dt)
			stepangular=dt*max_steps_angular
			print("MAX ANGULAR ACCEL/DECEL REACHED!!!!")
			print("new accel_angular:",stepangular/dt)
		v_initial.linear.x=v_initial.linear.x+step
		v_initial.angular.z=v_initial.angular.z+stepangular
		if(v_initial.linear.x>v_max):
			v_initial.linear.x=v_max
			
		self.vel_pub.publish(v_initial)
		timer=time.time()
		print('dt:',dt,'step_x:',step,'step_angular:',stepangular,'acceleration_x:',step/dt,'accel_angular:',stepangular/dt)
		return(v_initial,timer)				

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
		t_final=time.time()
		while(True):
			e_sum=e_sum+dist_prev
			dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
			dedt=dist-dist_prev	
			vel.linear.x=Kp*(dist)+Kd*dedt+Ki*e_sum
			vel.angular.z=6*Kp*(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
			
			
			
			vel_prev,t_final=self.step_vel(vel,vel_prev,t_final)
			#self.vel_pub.publish(vel)				
			t_initial=t_final
			self.rate.sleep()
			dist_prev=dist

			#vel_prev.linear.x=vel.linear.x
			#vel_prev.angular.z=vel_prev.angular.z
			
			
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
	def chase(self,target):
		
		dist=sqrt((self.position.x-target.x)**2+(self.position.y-target.y)**2)
		print("Current distance:",dist)
		if(dist<3):
			print('My Position:',self.position.x,self.position.y)
			print('Target Position:',target.x,target.y)
			print("Distance less than 3. Caught RT\n")
			self.rt_pose.unregister()
			rospy.set_param('caughtStatus',True)
		else:
			print("Moving now")
			self.go_to_goal(target)
	def plan(self,target):
		a=[]
		while(np.shape(a)[0]<3):
			a.append(target)
		center,radius=self.define_circle(a[0],a[1],a[2])
		print(center,radius)
	def define_circle(p1, p2, p3):
	    """
	    Returns the center and radius of the circle passing the given 3 points.
	    In case the 3 points form a line, returns (None, infinity).
	    """
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
			


x=turtle()
rospy.spin()

		

