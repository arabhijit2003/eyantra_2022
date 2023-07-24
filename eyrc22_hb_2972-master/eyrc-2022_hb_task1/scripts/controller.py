#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# for goal pose array
from geometry_msgs.msg import PoseArray

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

######## Set Initial Values #########
hola_x = 0
hola_y = 0
hola_theta = 0
ready=False  # will be used to check if goal poses are ready
ind=0
x_goals=[]
y_goals=[]
theta_goals=[]
#####################################

######## Callback Functions #########
def odometryCb(msg):
	# Write your code to take the msg and update the three variables
	global hola_x, hola_y, hola_theta
	hola_x=msg.pose.pose.position.x
	hola_y=msg.pose.pose.position.y
	hola_theta=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

	
def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals, ready

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()
	
	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)
	ready=True  # goal poses array ready

####################################

######## Utility Functions #########
def goal_reached(x,y,theta):
	# To check if Goal Pose reached by the Bot. x,y,theta are the errors in position from the goals
	return abs(x)<0.01 and abs(y)<0.01 and abs(theta)<0.01
	
####################################


def main():
	global x_goals,y_goals,ready, ind
	# Initialze Node
	# We'll leave this for you to figure out the syntax for
	# initialising node named "controller"
	rospy.init_node("controller")
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
	pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)  
	rospy.Subscriber("odom",Odometry, odometryCb)
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
	# Declare a Twist message
	vel = Twist()
	vel.angular.z=0
	vel.linear.x=0
	vel.linear.y=0
	# Initialise the required variables to 0
	# <This is explained below>
	
	# For maintaining control loop rate.
	rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	Kp=3
	#
	# 
	# Control Loop goes here
	#
	#
	while not rospy.is_shutdown():
		if not ready:
			# if Goal poses not recieved do nothing
			continue	
		#Set x_d, y_d, theta_d from the goal_list	
		x_d=x_goals[ind]
		y_d=y_goals[ind]
		theta_d=theta_goals[ind]

		# Find error (in x, y and theta) in global frame
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
		x=x_d-hola_x
		y=y_d-hola_y
		theta=theta_d-round(hola_theta,2)
		# (Calculate error in body frame)
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
		# 
		# This is probably the crux of Task 1, figure this out and rest should be fine.
		x,y=(x*math.cos(hola_theta)+y*math.sin(hola_theta)),(y*math.cos(hola_theta)-x*math.sin(hola_theta))

		# Check if goal reached. if reached increase index and wait for a moment to Stabilize
		if (goal_reached(x,y,theta) and ind<len(theta_goals)-1):					
			rospy.sleep(2)
			ind+=1
			
		# Finally implement a P controller 
		# to react to the error with velocities in x, y and theta.
		if (goal_reached(x,y,theta)):
			# If goal reached set velocities to 0
			vel_x=vel_y=vel_z=0
		else:
			# Otherwise set velocities based on P control system
			vel_x=x*Kp
			vel_y=y*Kp
			vel_z=theta*Kp*2

		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.

		vel.linear.x = vel_x
		vel.linear.y = vel_y
		vel.angular.z = vel_z

		pub.publish(vel)
		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
