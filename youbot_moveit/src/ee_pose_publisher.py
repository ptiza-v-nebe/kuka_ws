#!/usr/bin/env python  
import roslib
roslib.load_manifest('youbot_moveit')
import rospy
import math
import tf2_ros
import geometry_msgs.msg



if __name__ == '__main__':
    rospy.init_node('ee_pose_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    ee_pose_pub = rospy.Publisher('ee_pose', geometry_msgs.msg.Pose,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('virtual_link_1', 'gripper_tip', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        pose = geometry_msgs.msg.Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation.w = 1;
        ee_pose_pub.publish(pose)

        rate.sleep()