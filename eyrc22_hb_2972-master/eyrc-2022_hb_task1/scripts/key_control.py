#!/usr/bin/env python3

import rospy
from pynput.keyboard import *
from geometry_msgs.msg import Twist

def on_press(key):
    global av,yv,xv, lin_speed,ang_speed
    msg=Twist()
    try:
        if (key.char.lower()=="w"):
            yv=1
        elif (key.char.lower()=="s"):
            yv=-1
        if (key.char.lower()=="d"):
            xv=1
        elif (key.char.lower()=="a"):
            xv=-1
        if (key.char.lower()=="q"):
            av=1
        if (key.char.lower()=="e"):
            av=-1
    except AttributeError:
        pass
        
    msg.linear.y=yv*lin_speed
    msg.linear.x=xv*lin_speed
    msg.angular.z=av*ang_speed
    pub.publish(msg)

def on_release(key):
    global av,yv,xv
    msg=Twist()
    try:
        if (key.char.lower()=="w" or key.char.lower()=="s"):
            yv=0
        if (key.char.lower()=="a" or key.char.lower()=="d"):
            xv=0
        if (key.char.lower()=="q" or key.char.lower()=="e"):
            av=0
    except AttributeError:
        pass
    msg.linear.y=yv
    msg.linear.x=xv
    msg.angular.z=av
    pub.publish(msg)
    if (key==Key.esc):
        return False
    
    


if __name__=='__main__':
    yv=0
    xv=0
    av=0
    lin_speed=1
    ang_speed=2
    rospy.init_node("key_control_node")
    pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    
    with Listener(on_press=on_press,on_release=on_release) as listener:
        listener.join()
    rospy.spin()
        
        
    

    