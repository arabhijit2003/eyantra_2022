#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			hb#2972
# Author List:		Abhoy Sagar Bhowmik, Abhijit Roy, Dhruv Kumar, Harsha Gopisetti
# Filename:			task_0.py
# Functions:
# 					callback(), main()
# Nodes:		    /turtle1/cmd_vel


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
##############################################################


def callback(data):
    '''
	Purpose:
	---
	This function should be used as a callback. Refer Example #1: Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTE: Radius value should be 1. Refer expected output in document and make sure that the turtle traces "same" path.
	Input Arguments:
	---
        `data`  : []
            data received by the call back function
	Returns:
	---
        May vary depending on your logic.
	Example call:
	---
        Depends on the usage of the function.
	'''


def main():
    """
	Purpose:
	---
	This function will be called by the default main function given below.
    You can write your logic here.
	Input Arguments:
	---
        None
	Returns:
	---
        None
	Example call:
	---
        main()
	"""
    # initialize ros node
    rospy.init_node("node_turtle_revolve")

    # initializing the publisher to publish topic "/turtle/cmd_vel" as a message of datatype Twist
    pub=rospy.Publisher("/turtle1/cmd_vel",Twist, queue_size=10)

    # creating the message to be published
    msg=Twist()
    dis=0   # initialized an distance variable with zero value
    # setting the angular velocity as 1 rad/s and linear velocity as 1 unit/s
    # When both are equal it moves in circular path with radius 1 unit
    msg.angular.z=1
    msg.linear.x=1
    # storing the starting time as t_init will be used for distance calculation
    t_init=rospy.Time.now().to_sec()
    # Making the semi circle. we move until distance is 3.4 unit
    # It was supposed to be 3.14 unit or more accurately pi unit(since radius is 1)
    # However since time is consumed in interpreting the code and publishing messages
    # it has to be adjusted to 3.4 to get proper semicircle
    while dis<=3.4:
        t_now=rospy.Time.now().to_sec() #taking the current time instance
        dis=1*(t_now-t_init)   #calculating distance covered
        pub.publish(msg)  #publishing message with the angular and linear velocity
        print("My turtleBot is: Moving in circle!!")
        print(f'{dis:.2f}')

    # setting the angular velocity to 1 rad/s and linear velocity to 0. This will make it rotate
    msg.angular.z=1
    msg.linear.x=0

    # initialize angle as 0
    angle=0
    t_init=rospy.Time.now().to_sec()    #storing initial time in t_init
    # we will make it rotate untill angle is 1.5605 rad
    # it should be equal to 1.57 rad or more accurately pi/2 rad
    # but it gives more accurate results for 1.5605 because of reasons stated above
    while angle<1.5605:
        t_now=rospy.Time.now().to_sec()    #taking current time measurement
        angle=1*(t_now-t_init)     #calculating the angle
        pub.publish(msg)    #publishing the message with the angular velocity and linear velocity
        print("My turtleBot is: Rotating!")
        print(f'{-angle:.2f}')

    # setting linear velocity as 1 and angular velocity as 0 to make it move straight
    msg.angular.z=0
    msg.linear.x=1
    # again setting dis to 0
    dis=0
    t_init=rospy.Time.now().to_sec()    #taking the initial time
    # we will move the robot unitill it covers a distance of 2 unit
    while dis<=2:
        t_now=rospy.Time.now().to_sec()    #taking the current time reading
        dis=1*(t_now-t_init)   #calculating the distance covered
        pub.publish(msg)    #publishing the angular and linear velocity
        print("My turtleBot is: Moving straight!!!")
        print(f'{-angle:.2f}')

    # to stop the bot we give both linear and angular velocty to be 0
    msg.angular.z=0
    msg.linear.x=0
    pub.publish(msg)
    rospy.spin()    #this will prevent the node from exiting unitl node is shutdown but also
    # allow any ros instructions like subcriber callbacks pr publishing to occur

################# ADD GLOBAL VARIABLES HERE #################



##############################################################


################# ADD UTILITY FUNCTIONS HERE #################



##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")