#! /usr/bin/env python
import rospy
import rosservice
import random
from math import pi
import time

#If required, enter custom values to test system
x=random.random()*10
y=random.random()*10
theta=random.random()*pi

rospy.wait_for_service('/spawn')
print("Waiting for 10 seconds before spawning PT!")
time.sleep(10)
rosservice.call_service("/spawn",[x, y, theta, ""])

