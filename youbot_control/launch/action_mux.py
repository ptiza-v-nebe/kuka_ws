#!/usr/bin/python
import rospy
import actionlib
import actionlib_tutorials.msg

from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class youbot_action_mux:
    def __init__(self):
        self._as = actionlib.SimpleActionServer("/moveit/controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self.arm_client = actionlib.SimpleActionClient('/youbot/controller/manipulator_trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        self.base_client = actionlib.SimpleActionClient('/youbot/controller/base_trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.base_client.wait_for_server()
        self._as.start()

    def execute_cb(self, goal):
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.goal_time_tolerance = rospy.Time(1.0)
        arm_goal.trajectory.joint_names = ['arm_joint_1', 'arm_joint_2','arm_joint_3','arm_joint_4', 'arm_joint_5']
    
        for pt in goal.trajectory.points:
            pt_temp = JointTrajectoryPoint()
            pt_temp.positions = pt.positions[0:5]
            pt_temp.velocities = pt.velocities[0:5]
            pt_temp.accelerations = pt.accelerations[0:5]
            pt_temp.time_from_start = pt.time_from_start
            arm_goal.trajectory.points.append(pt_temp)

        base_goal = FollowJointTrajectoryGoal()
        base_goal.goal_time_tolerance = rospy.Time(1.0)
        base_goal.trajectory.joint_names = ['virtual_x', 'virtual_y','virtual_theta']
 
        for pt in goal.trajectory.points:
            pt_temp = JointTrajectoryPoint()
            pt_temp.positions.append(pt.positions[6])
            pt_temp.positions.append(pt.positions[7])
            pt_temp.positions.append(pt.positions[5])
            pt_temp.velocities.append(pt.velocities[6])
            pt_temp.velocities.append(pt.velocities[7])
            pt_temp.velocities.append(pt.velocities[5])
            pt_temp.accelerations.append(pt.accelerations[6])
            pt_temp.accelerations.append(pt.accelerations[7])
            pt_temp.accelerations.append(pt.accelerations[5])
            pt_temp.time_from_start = pt.time_from_start
            base_goal.trajectory.points.append(pt_temp)
        
        #rospy.loginfo(base_goal.trajectory.points[0].positions) #list of positions of one point
        #rospy.loginfo(base_goal.trajectory.points[1].positions)
        #rospy.loginfo(base_goal.trajectory.points[2].positions)

        if self._as.is_preempt_requested():
           rospy.loginfo("This action has been preempted")
           self._as.set_preempted()
           success = False 
           self._as.set_cancelled()
        else:
            #rospy.loginfo(base_goal.trajectory.points)
            self.base_client.send_goal(base_goal)
            self.arm_client.send_goal_and_wait(arm_goal)
            success = True

        if success:
            rospy.loginfo('Action Succeeded')
            self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('youbot_action_mux')
    server = youbot_action_mux()
    rospy.spin()