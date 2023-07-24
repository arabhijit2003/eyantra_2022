#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		HB#2972
# Author List:		Abhoy, Abhijit, Harsha, Dhruv
# Filename:		controller.py
# Functions:
#			signal_handeler, cleanup, task2_goals_Cb, aruco_feedback_Cb, inverse_kinematics, goal_reached, main
# Nodes:		/front_wheel_force,/left_wheel_force, /right_wheel_force, detected_aruco, task2_goals 


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []
d=1
hola_x = 0
hola_y = 0
hola_theta = 0
ready=False  # will be used to check if goal poses are ready
ind=0

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	pass
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
  
def task2_goals_Cb(msg):
	global ready
	global x_goals, y_goals, theta_goals
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
	ready=True

def aruco_feedback_Cb(msg):
	global hola_x, hola_y, hola_theta
	############ ADD YOUR CODE HERE ############
	hola_x=msg.x
	hola_y=msg.y
	hola_theta=msg.theta
	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################

def inverse_kinematics(w,v_x,v_y):
    global d
    uf=-d*w+v_x
    ur=-d*w-0.5*v_x-0.866*v_y
    ul=-d*w-0.5*v_x+0.866*v_y
    return uf,ur,ul

def goal_reached(x,y,theta):
	# To check if Goal Pose reached by the Bot. x,y,theta are the errors in position from the goals
	return abs(x)<1 and abs(y)<1 and abs(theta)<0.01

def main():
	global hola_x,hola_y,hola_theta, ind,x_goals, y_goals, theta_goals
	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   

	Kp=10
	vf=Wrench()
	vl=Wrench()
	vr=Wrench()
		
	while not rospy.is_shutdown():
		if not ready:
			continue
		# Calculate Error from feedback
		x_d=x_goals[ind]
		y_d=y_goals[ind]
		theta_d=theta_goals[ind]

		x=(x_d-hola_x)*0.2
		y=(y_d-hola_y)*0.2
		theta=theta_d-hola_theta

		# Change the frame by using Rotation Matrix (If you find it required)
		x,y=(x*math.cos(hola_theta)+y*math.sin(hola_theta)),(y*math.cos(hola_theta)-x*math.sin(hola_theta))

		# Calculate the required velocity of bot for the next iteration(s)
		if (goal_reached(x,y,theta)):
			vel_x=0
			vel_y=0
			vel_z=0
		else:		
			# Otherwise set velocities based on P control system
			vel_x=x*Kp
			vel_y=y*Kp
			vel_z=theta*Kp

		vf.force.x,vr.force.x,vl.force.x=inverse_kinematics(vel_z,vel_x,vel_y)
		
		# Apply appropriate force vectors
		right_wheel_pub.publish(vr)
		front_wheel_pub.publish(vf)
		left_wheel_pub.publish(vl)

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)
		if (goal_reached(x,y,theta) and ind<len(theta_goals)-1):					
			rospy.sleep(2)
			ind+=1

		rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

