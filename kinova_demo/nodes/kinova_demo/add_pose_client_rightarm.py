#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest
import kinova_msgs.msg


# use service to set torque control parameters	
def callsrv_addpose(CmdPoses):	
	service_address = '/right_arm_driver/in/add_pose_to_Cartesian_trajectory'	
	rospy.wait_for_service(service_address)
	try:
		client = rospy.ServiceProxy(service_address, AddPoseToCartesianTrajectory)
		client(CmdPoses)           
	except rospy.ServiceException as e:
		print ("Service call failed: %s"%e)
		return None


if __name__ == '__main__':
	# 创建节点
	rospy.init_node('add_pose_client_rightarm')
	
	n = 0
	# 给server发请求
	request = AddPoseToCartesianTrajectoryRequest()
	
	while (not rospy.is_shutdown()):
		request.X = -0.5
		request.Y = 0.2
		request.Z = 0.4 - (0.15 * (-math.cos(n)) + 0.15)
		request.ThetaX = -1.477
		request.ThetaY = -0.235
		request.ThetaZ = -1.385	
		print (request)
		callsrv_addpose(request)
		n = n + 1
		
		if n > 20:
			break
	
	rospy.spin()

