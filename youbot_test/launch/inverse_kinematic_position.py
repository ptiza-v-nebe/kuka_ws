#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_test')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
from std_srvs.srv import Empty
import control_msgs.msg  

from brics_actuator.msg import JointValue, JointPositions, JointVelocities



def main():
    pubPos = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=10)

    rospy.init_node('armcommand', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    jp = JointPositions()
    joints = []
    for i in range(0, 5):
         joints.append(JointValue())
	 joints[i].joint_uri = "arm_joint_" + str(i+1)
         joints[i].unit = "rad"

    while not rospy.is_shutdown():
	 joints[0].value = 3
	 joints[1].value = 1
	 joints[2].value = -1
	 joints[3].value = 1.85
	 joints[4].value = 3

         jp.positions = joints;

         pubPos.publish(jp)

         rate.sleep()
                        
if __name__ == '__main__':
    main()
