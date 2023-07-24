#!/usr/bin/env python3


# Team ID:		HB#2972
# Author List:		Abhoy, Abhijit, Harsha, Dhruv
# Filename:		controller.py
# Functions:
#			callback, main
# Nodes:		usb_cam/image_raw


######################## IMPORT MODULES ##########################

import numpy as np				# If you find it required
import rospy 
import socket
import signal		# To handle Signals by OS/user
import sys
import multitasking


from geometry_msgs.msg import Pose2D	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import cv2.aruco as aruco
import math		# Required to publish ARUCO's detected position & orientation



############################ GLOBALS #############################
pi=3.1415
marker_size=4
total_markers=250
key=getattr(aruco,f'DICT_{marker_size}X{marker_size}_{total_markers}')
aruco_dict=aruco.Dictionary_get(key)
aruco_param=aruco.DetectorParameters_create()
x_goals=[250,350,150,150,350]
y_goals=[250,300,300,150,150]
theta_goals=[0,pi/4,3*pi/4,-3*pi/4,-pi/4]

ready=True

hola_x=250
hola_y=250
hola_theta=0
ind=0

d=50
vf=0
vr=0
vl=0

ip = "192.168.232.1"     #Enter IP address of laptop after connecting it to WIFI hotspot


#We will be sending a simple counter which counts from 1 to 10 and then closes the socket
message=""

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup(s):
    s.close()
    print("cleanup done")


def callback(data):
	global hola_x,hola_y,hola_theta, ready
	hola_x=data.x
	hola_y=data.y
	hola_theta=data.theta
	# Calculate Error from feedback
	ready=True
			
	############ ADD YOUR CODE HERE ############
	


def inverse_kinematics(w,v_x,v_y):
	global d
	uf=-d*w+v_x
	ur=-d*w-0.5*v_x-0.866*v_y
	ul=-d*w-0.5*v_x+0.866*v_y
	return uf,ur,ul

def goal_reached(x,y,theta):
	# To check if Goal Pose reached by the Bot. x,y,theta are the errors in position from the goals
	return abs(x)<1 and abs(y)<1 and abs(theta)<0.02
	############################################
	  
def main():
	global hola_theta,hola_x,hola_y, ip, x_goals,y_goals,theta_goals, ind, fr, vf,vr,vl, ready
	rospy.init_node('controller')  
	rospy.Subscriber('hola_pose', Pose2D, callback)

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	
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

	Kp=2
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((ip, 8002))
		s.listen()
		conn, addr = s.accept()
		with conn:
			print(f"Connected by {addr}")
			while ind<len(x_goals):				
				if (not ready):
					continue		
				x_d=x_goals[ind]
				y_d=y_goals[ind]
				theta_d=theta_goals[ind]

				x=(x_d-hola_x)
				y=(y_d-hola_y)
				theta=theta_d-hola_theta
				print(f"Errors:{x},{y},{theta}")


				# Change the frame by using Rotation Matrix (If you find it required)
				x,y=(x*math.cos(hola_theta)+y*math.sin(hola_theta)),(y*math.cos(hola_theta)-x*math.sin(hola_theta))
				if (goal_reached(x,y,theta) and ind<len(theta_goals)-1):					
					rospy.sleep(3)
					ind+=1

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
				print(f"velocities:{vel_x},{vel_y},{vel_z}")
				vf,vr,vl=inverse_kinematics(vel_z,-vel_x,vel_y)
				# Bridge is Used to Convert ROS Image message to OpenCV image
		
				# Apply appropriate force vectors
				

				# Modify the condition to Switch to Next goal (given position in pixels instead of meters)
				
				message=f"{vf} {vr} {vl} \n"		
				conn.sendall(str.encode(message))
				rate.sleep()
		
	rospy.spin()
if __name__ == '__main__':
  main()
rospy.spin()