#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Wrench, Pose2D;

d=0.3

def InverseKinematics(w,v_x,v_y):
    global d
    uf=-d*w+v_x
    ur=-d*w-0.5*v_x-0.866*v_y
    ul=-d*w-0.5*v_x+0.866*v_y
    return uf,ur,ul

def aruco_feedback_Cb(msg):
    print(msg.x," , ", msg.y)


if __name__=="__main__":
    rospy.init_node("TestMotors")    
    f_w_pub=rospy.Publisher("/front_wheel_force", Wrench, queue_size=10)
    r_w_pub=rospy.Publisher("/right_wheel_force", Wrench, queue_size=10)
    l_w_pub=rospy.Publisher("/left_wheel_force", Wrench, queue_size=10)
    f_w_msg=Wrench()
    l_w_msg=Wrench()
    r_w_msg=Wrench()
    v_x=0
    v_y=0
    w=0
    rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
    vf,vr,vl=InverseKinematics(w,v_x,v_y)
    
    while not rospy.is_shutdown():  
        try:  
                    
            f_w_msg.force.x=vf
            r_w_msg.force.x=vr
            l_w_msg.force.x=vl
            f_w_pub.publish(f_w_msg)
            r_w_pub.publish(r_w_msg)
            l_w_pub.publish(l_w_msg)        
            

        except rospy.ROSInterruptException:   
            f_w_msg.force.x=0
            r_w_msg.force.x=0
            l_w_msg.force.x=0
            f_w_pub.publish(f_w_msg)
            r_w_pub.publish(r_w_msg)
            l_w_pub.publish(l_w_msg)

    



