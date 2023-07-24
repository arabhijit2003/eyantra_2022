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
# Filename:		feedback.py
# Functions:
#			callback, main
# Nodes:		detected_aruco, overhead_cam/image_raw


######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import cv2.aruco as aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
aruco_msg = Pose2D()
marker_size=4
total_markers=250
key=getattr(aruco,f'DICT_{marker_size}X{marker_size}_{total_markers}')
aruco_dict=aruco.Dictionary_get(key)
aruco_param=aruco.DetectorParameters_create()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	bbox,_,_=aruco.detectMarkers(get_frame,aruco_dict,parameters=aruco_param)
	
	pos=[0,0]
	if len(bbox)>0:
		for i in range(4):
			pos[0]+=0.25*bbox[0][0][i][0]
			pos[1]+=0.25*bbox[0][0][i][1]
		
		y=(bbox[0][0][3][1])-(bbox[0][0][0][1])
		x=(bbox[0][0][3][0])-(bbox[0][0][0][0])
		align=math.atan2(x,y)
		aruco_msg.x=pos[0]-639.5
		aruco_msg.y=-pos[1]+640.5
		aruco_msg.theta=align
		aruco_publisher.publish(aruco_msg)
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################
	  
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
if __name__ == '__main__':
  main()
