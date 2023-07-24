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
# Filename:		camfeedback.py
# Functions:
#			callback, main
# Nodes:		detected_aruco, usb_cam/image_raw


######################## IMPORT MODULES ##########################

import numpy as np			# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import cv2.aruco as aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################
hola_theta=0
hola_x=0
hola_y=0
frameReady=False
marker_size=4
total_markers=250
key=getattr(aruco,f'DICT_{marker_size}X{marker_size}_{total_markers}')
aruco_dict=aruco.Dictionary_get(key)
aruco_param=aruco.DetectorParameters_create()

posePub=rospy.Publisher("hola_pose", Pose2D, queue_size=10)

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	global hola_theta,hola_x,hola_y, posePub
	global get_frame, frameReady
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	K=np.array([[475.80105709602356, 0.0, 310.4264246388291],[ 0.0, 481.36276329196784, 232.5430107204745], [0.0, 0.0, 1.0]])
	D= np.array([[-0.4542026630944087], [0.15796605319292129], [0.004246103997990767], [0.006112923314298602]])
	DIM=(800,600)
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
	cal_frame=cv2.remap(get_frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)	
	current_frame=cv2.resize(cal_frame,(500,500),interpolation=cv2.INTER_LINEAR)
	bbox,ids,_=aruco.detectMarkers(current_frame,aruco_dict,parameters=aruco_param)
	arucos={}
	
	if (len(bbox)>0): 
		for j in range(len(bbox)):
			pos=[0,0]
			for i in range(4):
				pos[0]+=0.25*bbox[j][0][i][0]
				pos[1]+=0.25*bbox[j][0][i][1]
			arucos.update({ids[j][0]:pos})
			
	
	
	if 4 in arucos.keys() and 8 in arucos.keys() and 10 in arucos.keys() and 12 in arucos.keys():
		frameReady=True
	else:
		frameReady=False
		
	
	if not frameReady:
		return
	
	msg=Pose2D()
	matrix=cv2.getPerspectiveTransform(np.float32([arucos[4],arucos[8],arucos[10],arucos[12]]),np.float32([[0,0],[500,0],[500,500],[0,500]]))
	new_frame=cv2.warpPerspective(current_frame, matrix, (500, 500))

	bbox2,_,_=aruco.detectMarkers(new_frame,aruco_dict,parameters=aruco_param)
	
	if (len(bbox2)>0): 
		pos=[0,0]
		for i in range(4):
			pos[0]+=0.25*bbox2[0][0][i][0]
			pos[1]+=0.25*bbox2[0][0][i][1]
		align=math.atan2(bbox2[0][0][0][0]-bbox2[0][0][3][0],bbox2[0][0][3][1]-bbox2[0][0][0][1])
		hola_x=pos[0]
		hola_y=pos[1]
		hola_theta=align
		
		msg.x=hola_x
		msg.y=hola_y
		msg.theta=hola_theta
		posePub.publish(msg)
	disp_frame=cv2.resize(new_frame, (900,900), interpolation=cv2.INTER_LINEAR)

	cv2.putText(disp_frame,f"{hola_x},{hola_y}  {hola_theta}",(30,50),cv2.FONT_HERSHEY_PLAIN,2,(200,0,0),2,cv2.LINE_AA)
	cv2.imshow("Output",disp_frame)
	cv2.waitKey(1)
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
	rospy.init_node('cam_feedback_node')  
	rospy.Subscriber('usb_cam/image_raw', Image, callback)		
	rospy.spin()
if __name__ == '__main__':
  main()
