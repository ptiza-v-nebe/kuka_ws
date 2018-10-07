#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_test')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
from std_srvs.srv import Empty
import control_msgs.msg
import random

from brics_actuator.msg import JointValue, JointPositions, JointVelocities



def main():

    rospy.init_node('armcommand', anonymous=True)
    pubPos = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size=10)
    pubVel = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=10)
    rospy.sleep(2) #wait for connection
    
    rate = rospy.Rate(70) # 10hz
    
    #initialise positions
    jp = JointPositions()
    joints = []
    for i in range(0, 5):
         joints.append(JointValue())
	 joints[i].joint_uri = "arm_joint_" + str(i+1)
         joints[i].unit = "rad"
    
    #initialise velocities
    jv = JointVelocities()
    vels = []
    for i in range(0, 5):
         vels.append(JointValue())
	 vels[i].joint_uri = "arm_joint_" + str(i+1)
         vels[i].unit = "s^-1 rad"
    
    #go to home position
    for i in range(0,5):
         joints[i].value = 0.05

    joints[2].value = -joints[2].value
    jp.positions = joints;
    pubPos.publish(jp)
    rospy.sleep(5)


    #start main movement
    j = 1
    while not rospy.is_shutdown():
	 for i in range(0,5):
             joints[i].value = 0.008*j 

	 for i in range(0,5):
             vels[i].value = 0.0001*j + abs(random.random() * 0.05)

	 joints[2].value = -joints[2].value
         vels[2].value = -vels[2].value

         jp.positions = joints;
         jv.velocities = vels;

         #pubPos.publish(jp)
	 pubVel.publish(jv)

         j = j+1
         rate.sleep()
                        
if __name__ == '__main__':
    main()
