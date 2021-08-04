#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
import moveit_msgs.msg
import kinova_msgs.msg
import moveit_commander
from kinova_msgs.srv import *

class My_Obj(object):

  def __init__(self):
    super(My_Obj, self).__init__()
 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_sub', anonymous=True)
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    self.group = group

  def sub_and_execute(self, sub_goal):
      group = self.group

      joint_goal = group.get_current_joint_values()

      joint_goal[0] = sub_goal.joint1
      joint_goal[1] = sub_goal.joint2
      joint_goal[2] = sub_goal.joint3
      joint_goal[3] = sub_goal.joint4
      joint_goal[4] = sub_goal.joint5
      joint_goal[5] = sub_goal.joint6
      print joint_goal
      group.go(joint_goal, wait=True)
#      group.stop()
  def sub_waypoints(self, waypoints):
      group = self.group
      pass


  def cartesian_pose_client(self):
      rospy.wait_for_service('add_pose')
      pose = [[1,1,1,0,0,0.5],[0.5,1.5,1,0,-0.5,0.5]]

      try:
          pose_goal = AddPoseToCartesianTrajectoryRequest()
          pose_goal.X = pose[0][0]
          pose_goal.Y = pose[0][1]
          pose_goal.Z = pose[0][2]
          pose_goal.ThetaX = pose[0][3]
          pose_goal.ThetaY = pose[0][4]
          pose_goal.ThetaZ = pose[0][5]

          add_pose = rospy.ServiceProxy('add_pose', AddPoseToCartesianTrajectory)
          plan = add_pose(pose_goal)

          group = self.group
          group.execute(plan)

      except rospy.ServiceException as e:
          print("Service call failed: %s"%e)

    
def listener():

    joint_sub = My_Obj()
    rospy.Subscriber('my_joint_states',kinova_msgs.msg.JointAngles,joint_sub.sub_and_execute)

    rospy.spin()



if __name__ == '__main__':
#    listener()
    joint_sub = My_Obj()
    joint_sub.cartesian_pose_client()

