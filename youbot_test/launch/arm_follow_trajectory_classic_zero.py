#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_test')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
from std_srvs.srv import Empty
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from sensor_msgs.msg import JointState


def main():
    jta = actionlib.SimpleActionClient('/arm_1/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    jta.wait_for_server()
   
    goal = FollowJointTrajectoryGoal()                  
    goal.trajectory.joint_names = ['arm_joint_1', 'arm_joint_2','arm_joint_3','arm_joint_4', 'arm_joint_5']
    goal.goal_time_tolerance = rospy.Time(0.3)

    point = []
    for i in range(0, 4):
        point.append(JointTrajectoryPoint())
   
    point[0].positions =[0.1, 0.1, -0.1, 0.1, 0.1]
    point[0].velocities = [0,0,0,0,0]
    point[0].accelerations = [0,0,0,0,0]
    point[0].time_from_start = rospy.Duration(0.0)
    goal.trajectory.points.append(point[0])
        
    point[1].positions =[3.14, 1.57, -1.57, 1.57, 1.57]
    point[1].velocities = [0,0,0,0,0]
    point[1].accelerations = [0,0,0,0,0]
    point[1].time_from_start = rospy.Duration(6)
    goal.trajectory.points.append(point[1])
        
    point[2].positions =[2, 2, -2, 2, 2]
    point[2].velocities = [0,0,0,0,0]
    point[2].accelerations = [0,0,0,0,0]
    point[2].time_from_start = rospy.Duration(12.0)
    goal.trajectory.points.append(point[2])

    point[3].positions =[0.4, 0.4, -0.4, 0.4, 0.4]
    point[3].velocities = [0,0,0,0,0]
    point[3].accelerations = [0,0,0,0,0]
    point[3].time_from_start = rospy.Duration(18.0)
    goal.trajectory.points.append(point[3])

    jta.send_goal_and_wait(goal)
                        
if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
