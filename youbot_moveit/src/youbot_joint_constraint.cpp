#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

while(ros::ok()){
  move_group.clearPathConstraints();
  visual_tools.prompt("move to pose that initially satisfy joint constraints");
  std::vector<double> joint_group_positions1;
  joint_group_positions1.push_back(0.0);  // meter
  joint_group_positions1.push_back(0.0);  // meter
  joint_group_positions1.push_back(1.7);  // meter
  joint_group_positions1.push_back(3.0);  // arm_joint_1 (has to be specified)
  joint_group_positions1.push_back(1.9);  // arm_joint_2 (has to be specified)
  joint_group_positions1.push_back(-2.1); // arm_joint_3 (has to be specified)
  joint_group_positions1.push_back(0.6);  // arm_joint_4
  joint_group_positions1.push_back(0.6);  // arm_joint_5
  move_group.setJointValueTarget(joint_group_positions1);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan1;
  bool successjoint = (move_group.plan(joint_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("press next to plan and move to constrainted joint pose");
  //set joint constrains
  moveit_msgs::JointConstraint jc;
  moveit_msgs::Constraints constraints;
  jc.joint_name = "arm_joint_1";
  jc.position = 3.0;
  jc.tolerance_above = 0.1;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_2";
  jc.position = 1.6;
  jc.tolerance_above = 3.0;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_3";
  jc.position = -1.9;
  jc.tolerance_above = 1.5;
  jc.tolerance_below = 0.8;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  move_group.setPathConstraints(constraints);

  //configure, plan and move to pose goal
  geometry_msgs::Pose target_pose;
  target_pose.orientation = tf2::toMsg( tf2::Quaternion(tf2::Vector3(0,1,0), 0.785) );
  
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.4;
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
