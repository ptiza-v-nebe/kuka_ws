

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <moveit/kinematic_constraints/utils.h>

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

  //visual_tools.prompt("move to home pose");
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  //Robot takes each time new path to get to the goal, planner dont care on which way going to the goal
  //this needs to be contrainted to get more controlled movements
  while (ros::ok())
  {
    visual_tools.prompt("move to a pose on arbitrary way 1");
    //set target, plan and move
    geometry_msgs::Pose target_pose1;
    quaternionTFToMsg(tf::Quaternion(tf::Vector3(0, 0, 1), 0.785), target_pose1.orientation);
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.1;
    move_group.setPoseTarget(target_pose1);
    move_group.setPlanningTime(15.0);
    moveit::planning_interface::MoveGroupInterface::Plan target_plan1;
    bool target_success1 = (move_group.plan(target_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();

    visual_tools.prompt("move to a pose on arbitrary way 2");
    //set target, plan and move
    geometry_msgs::Pose target_pose2;
    quaternionTFToMsg(tf::Quaternion(tf::Vector3(0, 1, 1), 0.785), target_pose2.orientation);
    target_pose2.position.x = 0.3;
    target_pose2.position.y = 0.3;
    target_pose2.position.z = 0.3;
    move_group.setPoseTarget(target_pose2);
    move_group.setPlanningTime(15.0);
    moveit::planning_interface::MoveGroupInterface::Plan target_plan2;
    bool target_success2 = (move_group.plan(target_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
  }
  ros::shutdown();
  return 0;
}