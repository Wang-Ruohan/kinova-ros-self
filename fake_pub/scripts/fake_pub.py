#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('joint_states',JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        joint_states = (0.1, 0.2, 0.1, 0.5, 0, 1, 0, 0, 0, 0, 0, 0)
        rospy.loginfo(joint_states)
        pub.publish(joint_states)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass