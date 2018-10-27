#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "youbot";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("virtual_link_1");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Joint Constraint", rvt::WHITE, rvt::XLARGE);

  visual_tools.prompt("move to home pose");
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

while(ros::ok()){
  visual_tools.prompt("move to joint pose");
  //go to a pose with manually calculating IK and setting joints
  moveit::core::RobotStatePtr current_state1 = move_group.getCurrentState(); //I have current state
  geometry_msgs::Pose goal_pose_for_joints; // I have a goal pose where I wish to go
  tf::Quaternion qj1orient = tf::Quaternion(tf::Vector3(0,1,0), 0.785); //goal orientation
  quaternionTFToMsg(qj1orient, goal_pose_for_joints.orientation);
  goal_pose_for_joints.position.x = 0.2;
  goal_pose_for_joints.position.y = 0.2;
  goal_pose_for_joints.position.z = 0.15;
  std::vector<double> joint_group_positions1;

  bool success_ik = false;
  while(!success_ik){
   success_ik = current_state1->setFromIK(joint_model_group, goal_pose_for_joints);
  }
  ROS_INFO_STREAM("Success in IK? " << success_ik); 
  //current_state1->printStatePositions();
  current_state1->copyJointGroupPositions(joint_model_group, joint_group_positions1);
  for(int i = 0; i <joint_group_positions1.size(); i++)
    ROS_INFO_STREAM("jgp" << joint_group_positions1[i]);

  /*joint_group_positions1[0] = 0.0;  // radians
  joint_group_positions1[1] = 0.0;  // radians
  joint_group_positions1[2] = 1.7;  // radians
  joint_group_positions1[3] = 3.0;  // radians
  joint_group_positions1[4] = 1.7;  // radians
  joint_group_positions1[5] = 0.6;  // radians
  joint_group_positions1[6] = 0.6;  // radians
  joint_group_positions1[7] = 0.6;  // radians*/
  move_group.setJointValueTarget(joint_group_positions1);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan1;
  bool successjoint = (move_group.plan(joint_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("move to pose that satisfy constraints");
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.7;  // radians
  joint_group_positions[1] = 0.7;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  bool joint_success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("press next to plan and move to constrainted joint");

  //set joint constrains
  moveit_msgs::JointConstraint jc;
  moveit_msgs::Constraints constraints;
  jc.joint_name = "arm_joint_1";
  jc.position = 3.0;
  jc.tolerance_above = 0.1;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_3";
  jc.position = -1.9;
  jc.tolerance_above = 1.5;
  jc.tolerance_below = 0.8;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_2";
  jc.position = 1.6;
  jc.tolerance_above = 3.0;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  move_group.setPathConstraints(constraints);

  //set position constrains
  /*moveit_msgs::PositionConstraint pc;
  pc.link_name = "gripper_palm_link";
  //pc.header.frame_id = "virtual_link_1";
  pc.target_point_offset.x=0.2;
  pc.target_point_offset.y=0.2;
  pc.target_point_offset.z=0.2;
  shape_msgs::SolidPrimitive box;
  box.type = 1;  
  box.dimensions.push_back(1.2);
  box.dimensions.push_back(1.2);
  box.dimensions.push_back(3.0);
  pc.constraint_region.primitives.push_back(box);
  geometry_msgs::Pose primPose;
  primPose.orientation.w = 1.0;
  primPose.position.x = 0.0;
  primPose.position.y = 0.0;
  primPose.position.z = 0.0;
  pc.constraint_region.primitive_poses.push_back(primPose);
  pc.weight = 1;*/
  //constraints.position_constraints.push_back(pc);
  //move_group.setPathConstraints(constraints);

  //orientation constraint
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "gripper_tip";
  ocm.header.frame_id = "virtual_link_1";
  //get actual orientation
  //geometry_msgs::PoseStamped actual_pose = move_group.getCurrentPose();
  tf::Quaternion q_rot_constr = tf::Quaternion(tf::Vector3(0,1,0),0.785);
  quaternionTFToMsg(q_rot_constr, ocm.orientation);
  //ocm.orientation = actual_pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = 2.0;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 2.0;
  ocm.weight = 1.0;
  //constraints.orientation_constraints.push_back(ocm);
  //move_group.setPathConstraints(constraints);
  
  //configure, plan and move to pose goal
  geometry_msgs::Pose target_pose;
  tf::Quaternion q_rot = tf::createQuaternionFromRPY(3.14159265359, 0, 0);
  tf::Vector3 vec3(0,1,0);
  q_rot.setRotation(vec3,0.785);
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
