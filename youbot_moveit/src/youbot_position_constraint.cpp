#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <moveit/kinematic_constraints/utils.h>

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

  //visual_tools.prompt("move to home pose");
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  while(ros::ok()){
  
  visual_tools.prompt("move to a pose on arbitrary way");
  move_group.clearPathConstraints();
  //set target, plan and move
  geometry_msgs::Pose target_pose1;
  quaternionTFToMsg(tf::Quaternion(tf::Vector3(0,0,1),0.785), target_pose1.orientation);
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.3;
  target_pose1.position.z = 0.1;
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan target_plan1;
  bool target_success1 = (move_group.plan(target_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("move to position constrained pose");
  //create target
  geometry_msgs::Pose target_pose;
  quaternionTFToMsg(tf::Quaternion(tf::Vector3(0,1,0),0.785), target_pose.orientation);
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.4;
  target_pose.position.z = 0.1;
  
  //construct constrains
  geometry_msgs::PointStamped point_stamped;
  point_stamped.point = target_pose.position;
  point_stamped.header.frame_id = "virtual_link_1"; //reference frame for the bounding box
  float tolerance = 0.3;
  move_group.setPathConstraints(kinematic_constraints::constructGoalConstraints ("gripper_tip", point_stamped, tolerance));

  //show constrains
  Eigen::Affine3d wcuboid_pose = Eigen::Affine3d::Identity();
  wcuboid_pose.translation().x() += target_pose.position.x;
  wcuboid_pose.translation().y() += target_pose.position.y;
  Eigen::Vector3d min_point, max_point;
  min_point << -tolerance, -tolerance, 0;
  max_point << tolerance, tolerance, tolerance;
  visual_tools.publishWireframeCuboid(wcuboid_pose, min_point, max_point, rvt::GREEN);
  visual_tools.trigger(); 
  

  //set target, plan and move
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
