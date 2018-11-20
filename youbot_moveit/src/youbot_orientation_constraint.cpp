#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "youbot";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("virtual_link_1");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  while (ros::ok())
  {
    visual_tools.prompt("move to pose in origin with zero rotation");
    geometry_msgs::Pose target_pose11;
    tf::Quaternion q_rot11 = tf::createQuaternionFromRPY(0, 0, 0);
    tf::Vector3 vec311(1, 0, 0);
    q_rot11.setRotation(vec311, 0.0);
    quaternionTFToMsg(q_rot11, target_pose11.orientation);
    target_pose11.position.x = 0.0;
    target_pose11.position.y = 0.0;
    target_pose11.position.z = 0.25;
    move_group.setPoseTarget(target_pose11);
    move_group.setPlanningTime(15.0);
    moveit::planning_interface::MoveGroupInterface::Plan target_plan11;
    bool target_success11 = (move_group.plan(target_plan11) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    visual_tools.prompt("move to pose that satisfy constraints, manually calculate ik");
    //go to a pose with manually calculating IK and setting joints
    moveit::core::RobotStatePtr current_state1 = move_group.getCurrentState(); //I have current state
    geometry_msgs::Pose goal_pose_for_joints;                                  // I have a goal pose where I wish to go
    //tf::Quaternion quat_y = tf::Quaternion(tf::Vector3(0,1,0), 0.785); //rotate first about y by 45 degree
    //tf::Quaternion quat_z = tf::Quaternion(tf::Vector3(0,0,1), 1.5707); // then rotate about z by 90 deg
    //tf::Quaternion quat_res = quat_z*quat_y;

    Eigen::Quaterniond qf2v = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, -1));
    Eigen::Quaterniond offset = Eigen::Quaterniond(Eigen::AngleAxisd(-2.0, Eigen::Vector3d(0, 1, -1)));
    tf::Quaternion quat_res = tf::Quaternion(tf::Vector3(1, 0, 0), 0.0);
    tf::quaternionEigenToTF(offset * qf2v, quat_res);
    quaternionTFToMsg(quat_res, goal_pose_for_joints.orientation);
    goal_pose_for_joints.position.x = 0.2;
    goal_pose_for_joints.position.y = 0.2;
    goal_pose_for_joints.position.z = 0.15;
    std::vector<double> joint_group_positions1;

    bool success_ik = false;
    while (!success_ik)
    {
      success_ik = current_state1->setFromIK(joint_model_group, goal_pose_for_joints);
    }
    ROS_INFO_STREAM("Success in IK? " << success_ik);
    //current_state1->printStatePositions();
    current_state1->copyJointGroupPositions(joint_model_group, joint_group_positions1);
    for (int i = 0; i < joint_group_positions1.size(); i++)
      ROS_INFO_STREAM("joint_group_positions1[" << i << "]: " << joint_group_positions1[i]);

    move_group.setJointValueTarget(joint_group_positions1);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan1;
    bool successjoint = (move_group.plan(joint_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    visual_tools.prompt("press next to plan and move to constrainted joint");

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constraints.orientation_constraints[0];
    ocm.link_name = "gripper_tip";
    ocm.header.frame_id = "virtual_link_1";
    tf::Quaternion c_quat(tf::Vector3(0, 1, 0), 0.785);
    quaternionTFToMsg(c_quat, ocm.orientation);
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 1.0;
    ocm.weight = 1.0;

    /*tf::Quaternion c_quat(tf::Vector3(0,1,0),0.785);
  geometry_msgs::QuaternionStamped c_quat_geo;
  c_quat_geo.header.frame_id = "virtual_link_1";
  quaternionTFToMsg(c_quat, c_quat_geo.quaternion);
  moveit_msgs::Constraints constraints = kinematic_constraints::constructGoalConstraints ("gripper_tip", c_quat_geo, 1.0);*/
    move_group.setPathConstraints(constraints);

    //configure, plan and move to pose goal
    geometry_msgs::Pose target_pose;
    tf::Quaternion q_rot = tf::createQuaternionFromRPY(3.14159265359, 0, 0);
    tf::Vector3 vec3(0, 1, 0);
    q_rot.setRotation(vec3, 0.785);
    quaternionTFToMsg(q_rot, target_pose.orientation);
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.1;
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(15.0);
    moveit::planning_interface::MoveGroupInterface::Plan target_plan;
    bool target_success = (move_group.plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    //clean up all constraints
    move_group.clearPathConstraints();
  }
  ros::shutdown();
  return 0;
}
