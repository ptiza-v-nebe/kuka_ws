#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_moveit')

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import sys

def prepare():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("youbot")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    
    #joint space
    joint_goal = group.get_current_joint_values()
    joint_goal[3] = 3
    group.go(joint_goal, wait=True)
    group.stop()

    pose = group.get_current_pose().pose
    print pose
    q = (
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w)
    e = tf.transformations.euler_from_quaternion(q)
    print e
    pose = geometry_msgs.msg.Pose()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    pose.position.x = 0.659 #0.55
    pose.position.y = 0.0
    pose.position.z = 0.32 #0.35
    print pose
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
def cartesianPath():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("youbot")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    if 1:
        waypoints = []
        scale=1

        wpose = group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0) # jump_threshold
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);
        group.execute(plan, wait=True)
def goHome():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("youbot")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    group.set_named_target("home")
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
#setJointGoal()
#setPoseGoal()
    if sys.argv[1] == '0':
        prepare()
    elif sys.argv[1] == '1':
        cartesianPath()
    elif sys.argv[1] == '2':
        goHome()