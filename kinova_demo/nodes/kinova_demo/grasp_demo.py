#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: HOnghao Lv lvhonghao@zju.edu.cn

import rospy
import threading
import time
import tf

from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest
import kinova_msgs.msg
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from tf.transformations import *
from math import pi

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

# All sizes in m!
hz = 10 # hz

# 控制机器人到达目标位姿pose
def cartesian_pose_client(position, orientation, prefix):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(200.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

# 抓取目标位姿发送程序
def object_tfpub():
    rate = rospy.Rate(200) # 200hz
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        br.sendTransform((0.2,0.1,0),(0,0,0,0),rospy.Time.now(),"object","ar_marker_0")
        rate.sleep()

def run():
    """Starts the node
    Runs to start the node and initialize everthing. Runs forever via Spin()
    :returns: Nothing
    :rtype: None
    """
    # 初始化节点
    rospy.init_node('grasp_demo')
    global rate
    rate = rospy.Rate(100) # 100hz

    # Subcribe the topic from RV2 3 motors
    rospy.Subscriber('/joint_states_motor',JointState,RV2_motorjointstate_callback)
    # Subcribe the topic from kinova from RV2
    rospy.Subscriber('/right_arm_driver/out/tool_pose', PoseStamped, get_current_pose_r_callback)
    # Publish the Joy-type sensor data
    
    global command_vel_pub_l , command_vel_pub_r, command_vel_pub_m, command_pos_pub_m 
    command_vel_pub_l = rospy.Publisher('/left_arm_neuron_vel_teleop', Joy, queue_size = 100, latch=True)
    command_vel_pub_r = rospy.Publisher('/right_arm_neuron_vel_teleop', Joy, queue_size = 100, latch=True)
    command_vel_pub_m = rospy.Publisher('/motor_neuron_vel_teleop', Joy, queue_size = 100, latch=True)
    command_pos_pub_m = rospy.Publisher('/motor_control/input/position', JointState, queue_size = 100, latch=True)

    time.sleep(3)
  
    # 定义并启动各子进程
    t_object_tfpub = threading.Thread(target = object_tfpub)
    t_object_tfpub.start()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
        print("##Program finished##")        
    except rospy.ROSInterruptException:
        pass
