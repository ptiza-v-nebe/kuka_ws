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

def setPoseGoal():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("youbot")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    pose_on_ground = geometry_msgs.msg.PoseStamped()
    pose_on_ground.header.frame_id = '/virtual_odom'
    pose_on_ground.pose.position.x = 0.0
    pose_on_ground.pose.position.y = 0.0
    pose_on_ground.pose.position.z = -0.05

    scene.add_mesh('ground',pose_on_ground,"/home/zarevich/Documents/kuka_ws/src/youbot_moveit/model/787/B_787_8.dae")
    
    #joint space
    joint_goal = group.get_current_joint_values()
    joint_goal[3] = 3
    #group.go(joint_goal, wait=True)
    #group.stop()
    
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose = geometry_msgs.msg.Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    pose.position.x = 0.4
    pose.position.y = 0.2
    pose.position.z = 0.3
    group.set_pose_target(pose)
    #plan = group.go(wait=True)
    #group.stop()
    #group.clear_pose_targets()

if __name__ == '__main__':
  setPoseGoal()