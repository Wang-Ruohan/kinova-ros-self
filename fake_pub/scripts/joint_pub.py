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




""" Global variable """
class My_Obj(object):

  def __init__(self):
    super(My_Obj, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_pub', anonymous=True)
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

#    joint_pub = rospy.Publisher('my_joint_states',kinova_msgs.msg.JointAngles,queue_size=20)
    joint_pub = rospy.Publisher('my_joint_states',kinova_msgs.msg.JointAngles,queue_size=20)

    self.group = group
    self.joint_pub = joint_pub

  def my_pub(self):

 #    delta = [-0.01, 0.01, 0.01, -0.01, 0.01, 0.01]
      rate = rospy.Rate(10) # 10hz
      group = self.group
      joint_publisher = self.joint_pub
      joint_goal = kinova_msgs.msg.JointAngles()

      while not rospy.is_shutdown():
          goal = group.get_current_joint_values()
          goal = [i + 0.2 for i in goal]
          rospy.loginfo(goal)
          joint_goal.joint1 = goal[0]
          joint_goal.joint2 = goal[1]
          joint_goal.joint3 = goal[2]
          joint_goal.joint4 = goal[3]
          joint_goal.joint5 = goal[4]
          joint_goal.joint6 = goal[5]
          joint_publisher.publish(joint_goal)
          rate.sleep()
  

  def pub_cartesian_path(self, scale = 1):
      pass

      group = self.group
      waypoints = []

      wpose = group.get_current_pose().pose
      wpose.position.z += scale * 0.1  # First move up (z)
      wpose.position.y += scale * 0.2  # and sideways (y)
      wpose.position.x -= scale * 0.1
#     waypoints.append(copy.deepcopy(wpose))
         
      (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
  
  def Add_Pose(self,req):
      group = self.group
      goal = geometry_msgs.msg.Pose()
      goal.position.x = req.X
      goal.position.y = req.Y
      goal.position.z = req.Z
      q = tf.transformations.quaternion_from_euler(req.ThetaX, req.ThetaY, req.ThetaZ)
      goal.orientation.x = q[0]
      goal.orientation.y = q[1]
      goal.orientation.z = q[2]
      goal.orientation.w = q[3]

      group.set_pose_target(goal)
      plan = group.plan().joint_trajectory
      return AddPoseToCartesianTrajectoryResponse(plan)

  def add_pose_server(self):
      s = rospy.Service('add_pose',AddPoseToCartesianTrajectory, self.Add_Pose)
      rospy.spin()
  
if __name__ == '__main__':
    pub = My_Obj()
#    pub.my_pub()
    pub.add_pose_server()

    

