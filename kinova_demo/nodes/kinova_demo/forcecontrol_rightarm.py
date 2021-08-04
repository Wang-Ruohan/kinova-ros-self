#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import time
from kinova_msgs.srv import *
import kinova_msgs.msg
import geometry_msgs.msg


def publishForceCmd(force_cmds, duration_sec, prefix):
    #use service to set torque control parameters	
    service_address = '/' + prefix + 'driver/in/set_torque_control_parameters'	
    rospy.wait_for_service(service_address)
    try:
        setTorqueParameters = rospy.ServiceProxy(service_address, SetTorqueControlParameters)
        setTorqueParameters()           
    except rospy.ServiceException, e:
        print "1 Service call failed: %s"%e
        return None	

    #use service to switch to torque control	
    service_address = '/' + prefix + 'driver/in/set_torque_control_mode'	
    rospy.wait_for_service(service_address)
    try:
        switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
        switchTorquemode(1)         
    except rospy.ServiceException, e:
    	print "2 Service call failed: %s"%e
    	return None	

    time.sleep(1)
    #publish joint torque commands
    topic_name = '/' + prefix + 'driver/in/cartesian_force'
    pub = rospy.Publisher(topic_name, kinova_msgs.msg.CartesianForce, queue_size=1)
    force = kinova_msgs.msg.CartesianForce()
    force.force_x = force_cmds[0]
    force.force_y = force_cmds[1]
    force.force_z = force_cmds[2]
    force.torque_x = force_cmds[3]
    force.torque_y = force_cmds[4]
    force.torque_z = force_cmds[5]
    count = 0		
    rate = rospy.Rate(100)
    L = []
    thread.start_new_thread(input_thread, (L,))
    while (count < 100*duration_sec):
        count = count + 1		
        pub.publish(force)
        rate.sleep()
        print count
        if L: break

    #use service to switch to position control	
    try:           
        switchTorquemode(0)
        return None
    except rospy.ServiceException, e:
        print "3 Service call failed: %s"%e
        return None

if __name__ == '__main__':

    global forceCmd
    rospy.init_node('force_control')

    forceCmd = [0, 1, 0, 0, 0, 0]

    duration = 25
    prefix = 'right_arm_'

    publishForceCmd(forceCmd, duration, prefix)

    rospy.spin()




