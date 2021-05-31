# Turtlesim_operations

This repository models a leader-follower bots using turtlesim package under various conditions such as intermittent communication, noisy communication. It implements a planning element for a slower follower bot to catch up with the faster leader bot. Report in PDF can be found in [this report](https://github.com/niteshjha08/Turtlesim_operations/edit/master/Report_final.pdf)


## Setup :
1. ROS Melodic
2. Ubuntu 18.04

## Tasks
### Task 1: Navigate the turtlebot from a random location to an input location without overshooting and using PID control.
To reach the input goal, constants of proportional, derivative and integral gains are defined. The respective errors(p,d,i) are calculated in loops and velocity is published, until the self position differs from the goal by a specified tolerance. Adjusting this value (currently 0.5), we can achieve more precise goals. Here, the parameter for setting linear velocity is taken as distance (always positive) from goal. It could also be (current_coordinate - goal_coordinate), hence including reverse velocities also. However, this can also be resolved by setting appropriate angular velocities, which is what has been done here. Also, the derivative error has not been divided by time as the derivative constant would account for that.
The launch file [turtle_nav.launch](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/launch/turtle_nav.launch) to reproduce the results. 
Here, target positions were given in order : (1,1), (10,9.5), (10,3.4), (1.4,9)
Position tolerance: 0.5 units 
![goal1_gif](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal1_gif.gif)
Here is the [full video](https://drive.google.com/file/d/1keQcUVyqBggQXRcfeCemUhhf8-HpUMrL/view)
Result plot:
![goal1](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal1.png)
[Here](https://drive.google.com/file/d/1lUXga59tx0Sdz5LdBZiTxlYbsAQ85CT0/view) is the video of verifying result positions

### Task 2: Implement maximum accelerations and form a grid path
To limit the accelerations, a function step_vel has been used. In this, the target velocity is sent, and it increases the velocity in steps. Before publishing this velocity, however, the step is checked for its value.
The maximum acceleration limits are defined, and can be changed. The delta(time) is recorded for each stage of velocity increment. If the velocity step/delta(time) is greater than the maximum value, the step size is reset to maximum allowable value
w.r.t. that particular delta(time). This process is done for both linear and angular acceleration/deceleration.
![goal1](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal2.PNG)
The gradual rise of the velocities (linear,angular) are shown in the figure. The node for this is `goal_accel_decel.py`
Implementation and result curve is shown [here](https://drive.google.com/file/d/17wUH6rEMyw2ZPY3DMOKhlmDHWdiCQKIG/view)

*Grid*
The grid task can be though as discrete sections.
1.	From any random location, go to the starting position in fastest way possible.
2.	Thereafter, grid points are defined which have to be followed. Hence, fastest way would give a curvature and not follow lines. Thus, grid corners are defined.
3.	However, at the end of a corner, the orientation of the turtle is perpendicular to next line that has to be followed. Directly commanding to go the next corner would again give a curvature. (Note: The lines are given importance, if that wasn’t the case, subsequent calls to go_to_goal(target) would suffice.)
 
4.	To avoid curvature at the end, a rotate() function has been defined. This takes in an angle and rotates it with proportional control.
5.	Finally, the grid function is defined. Grid_corners contain x,y coordinates, as well as angles to be rotated at the end of traversal. Go_to_goal() is then called.
The acceleration parameters are varied and trajectory recorded for them

![goal2_grid](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal2_grid.gif)

[Here](https://drive.google.com/file/d/1OFy4_0Em83y32akhVC60_gH7-NU7M5I2/view) is the full video of the resulting grid
Grid 1: Max_accel_linear: 1500, Max_accel_angular:6000
![grid1](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal2_grid1.png)

Grid 2: Max_accel_linear: 2500, Max_accel_angular:8000
![grid1](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal2_grid2.png)

Grid 3:Max_accel_linear: 12000 Max_accel_angular:12000
![grid1](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal2_grid3.png)


### Task 3: Take user input radius and velocity limit to form circular path with the turtlebot
The circles() function prompts the user for input of radius and velocity, accordingly calculating angular velocity.
Once determined, the velocity of the turtle is increased in steps using step_vel() function.
When launched, it starts a time variable. Every 5 seconds, this variable causes the node to publish two poses: rt_real_pose, which is the actual pose of the turtle, and rt_noisy_pose, which contains random gaussian noise added. The mean and  standard deviation of this noise is a major parameter in determining whether it can be chased down in further goals.
![goal3](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal3_gif.gif)

[Here](https://drive.google.com/file/d/1lbynjAezwX4CGv5MF60ctPDxAcKCbkYM/view) is the link to the full video


### Task 4: Implement a leader-follower chase with two turtlebots
The turtle from Task 3 is the the leader, and the turtle from task 2 is the follower. The leader goes in circles and the follower, with limited acceleration and velocity has to reach within a threshold distance of the leader. 
First, the turtlesim_node is launched with one turtle.
Then the circle function from Goal 3 is used to start circling motion of this leader turtle (also called RT). Then another node turtlespawn(called PT) is used. This first gives a delay of 10 seconds. After this, it calls the service spawn and gives random values to spawn another turtle, which gets named ‘turtle2’. However, the name could be taken as a return argument of the service also, but assuming that only one of the launch files are run at a time, there won’t be any name mismatch. The launch file then spawns the chase_limit_accel.
This node subscribes to the rt_real_pose. Hence, it has to wait for 5 seconds before receiving its first goal position. The speed of this is greater than that, and as such no limit on speed has been imposed at this stage (will be done later).
However, the success of this program is contingent on various factors. The chase was said to be completed when the turtles were 10 units apart. However, this resulted In no motion most of the times and were already less than 10 units apart. Hence the new threshold was taken as 3 units.

Due to the increased speed, PT can reach RT’s last location quickly. But the  difference of the speeds determine exactly how far apart will they be when the next rt_real_pose comes in. Till then the PT will rest on the last known location of RT if it reaches in less than 5 seconds. In this case, there are several possibilities. If the circle is large enough, the RT might be on the other end after 5 seconds. While PT again reaches the new location, RT could be>3 units far. Depending on how much it is catching up every 5 seconds, the chase would then be completed in finite time. However, if PT is very fast, it might also catch up in one go. Radius of circle will play a role, as a smaller one finishes the task quickly.
![goal4_gif](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal4_gif.gif)
[Here](https://drive.google.com/file/d/1RDmeeTQgVahchoQr88Pp8beO4Pa8VEZQ/view) is the full video of the chase.
Here, the radius of the circle is 2 units, and velocity of RT is 2 units. PT has limit on acceleration, not on speed.
In this example, it took one rt_real_pose to reach within 3 units of RT
![goal4](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal4.png)
dx=2 units, dy=0.9 units.


### Task 5: Repeat the chase with the follower having lesser velocity than the leader with a planning element
In the last example, due to greater velocity, PT would always gain on RT, even if in very small increments, as it would reach its last location quickly and RT would not have left the last location as far behind. So in the next position message, it could shorten their distance even more. Thus we could be sure that it will catch up in future.
However, if the velocity of RT is double that of PT, whenever PT reaches a location  RT has been, RT would have traversed more than what PT could catch. Hence we cannot be sure of catching up in this case.
 
However, this is true for a generic path. For a circle, as the RT would at regular intervals come at the same location it had previously been, the gradient of their velocities have the same effect of closing the gap between them.
Below is a video of this. Here, the velocity of RT is 2, and that of PT is limited at 1. 
![goal5_gif](https://github.com/niteshjha08/Turtlesim_operations/blob/master/turtlesim_operations/images/goal5_gif.gif)
[Here](https://drive.google.com/file/d/12yBbskAqIvOFApM_Vo9qfZlZIFaAre8v/view) is the full video of implemention of this method.

### Task 6: Modification of the chase, where the follower can only access leader's position with Gaussian noise and the communication is intermittent (every 5 seconds)
The RT sends a noisy pose on topic rt_noisy_pose. The uncertainty of reaching near the target is dependent on the mean and standard deviation of the gaussian noise. If high values of standard deviation are chosen, with the PT half as fast as RT, it will not be able to catch RT. However, again due to  circular motion, this may be the case, as even with gaussian noise, and a threshold of 3 units, the PT would get close enough to end the chase. As random gaussian noise fluctuates quickly, it might destabalize PT.

[Here](https://drive.google.com/file/d/1E8JnyZ2E-_vz9wWivOzUkm1Lj5qxVKhN/view) is a video where PT was able to approach RT (std. dev=1) and get in proximity  to end the chase



