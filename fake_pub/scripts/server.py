#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import geometry_msgs
import sensor_msgs.msg
import kinova_msgs.msg
from kinova_msgs.srv import *
import tf

 
def Add_Pose(req):
    # group_name = "arm"
    # group = moveit_commander.MoveGroupCommander(group_name)

    moveit_commander.roscpp_initialize(sys.argv)
    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)
    i = 0 
    wpose = []
    while 1:
        q = tf.transformations.quaternion_from_euler(req.ThetaX, req.ThetaY, req.ThetaZ)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = req.X
        pose.position.y = req.Y
        pose.position.z = req.Z
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        wpose.append(pose)
        i += 1

        if i == 3 * i/3:
            (plan, fraction) = group.compute_cartesian_path(wpose, 0.01, 0)
            group.execute(plan, wait = True)
#            clear = rospy.Service('add_pose', ClearTrajectories) 
            wpose = []


def add_pose_server():
    rospy.init_node('server', anonymous=True)
    s = rospy.Service('add_pose',AddPoseToCartesianTrajectory, Add_Pose)
    rospy.spin()

if __name__ == '__main__':
    add_pose_server()




