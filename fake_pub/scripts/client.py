#! /usr/bin/env python

#import roslib; roslib.load_manifest('kinova_demo')
import rospy
import kinova_msgs.msg
import geometry_msgs.msg
import tf 
import std_msgs.msg
import math
from kinova_msgs.srv import *
import moveit_commander
import sys

def cartesian_pose_client():
    rospy.wait_for_service('add_pose')
    try:
        pose_goal = AddPoseToCartesianTrajectoryRequest()
        delta = [0.3, 0.3, 0.2, 0.01, 0.01, 0.01]
        i = 0
        while i < 20:
            pose =  [0.57, 0.33, 0.74, 0.37, 0.18, 0.37, 0.82]
            pose_goal.X = pose[0] + delta[0]
            pose_goal.Y = pose[1] + delta[1] 
            pose_goal.Z = pose[2] + delta[2] 
            pose_goal.ThetaX = pose[3] + delta[3]
            pose_goal.ThetaY = pose[4] + delta[4]
            pose_goal.ThetaZ = pose[5] + delta[5]
            add_pose = rospy.ServiceProxy('add_pose', AddPoseToCartesianTrajectory)
            add_pose(pose_goal)  #there is no response!
            i += 1

        rospy.spin()
        # group = moveit_commander.MoveGroupCommander('arm')
        # group.execute(plan)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    cartesian_pose_client()


        




        
        
        