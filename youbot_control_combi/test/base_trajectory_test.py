#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_control_combi')

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
    jta = actionlib.SimpleActionClient('/youbot/controller/base_trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
    jta.wait_for_server()
   
    goal = FollowJointTrajectoryGoal()                  
    goal.trajectory.joint_names = ['virtual_x', 'virtual_y','virtual_theta']
    goal.goal_time_tolerance = rospy.Time(0.3)

    point = []
    for i in range(0, 3):
        point.append(JointTrajectoryPoint())
   
    point[0].positions =[0.0, 0.0, 0.0]
    point[0].velocities = [0,0,0]
    point[0].accelerations = [0,0,0]
    point[0].time_from_start = rospy.Duration(0.0)
    goal.trajectory.points.append(point[0])

    point[1].positions =[0.4, 0.4, 1.57]
    point[1].velocities = [0,0,0]
    point[1].accelerations = [0,0,0]
    point[1].time_from_start = rospy.Duration(5.0)
    goal.trajectory.points.append(point[1])

    point[2].positions =[0.0, 0.0, 0.0] #aus irgendeinem Grund Zahlen fuer Winkel um 1.0 und hoeher schaukeln sich auf, das Problem ist vielleicht nicht Theta sondern x und y
    point[2].velocities = [0,0,0]
    point[2].accelerations = [0,0,0]
    point[2].time_from_start = rospy.Duration(10.0)
    goal.trajectory.points.append(point[2])


    jta.send_goal_and_wait(goal)
                        
if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()