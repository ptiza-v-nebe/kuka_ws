#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/youbot/controller/youbot_trajectory/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  }
  else
  {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}

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

  visual_tools.prompt("Move to home pose");
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  while (ros::ok())
  {
    move_group.clearPathConstraints();

    visual_tools.prompt("move to pose that initially satisfy joint constraints");
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> initial_positions;
    current_state->copyJointGroupPositions(joint_model_group, initial_positions);
    initial_positions[3] = 3.0;
    move_group.setJointValueTarget(initial_positions);
    move_group.move();

    moveit_msgs::JointConstraint jc;
    moveit_msgs::Constraints constraints;
    jc.joint_name = "arm_joint_1";
    jc.position = 3.0;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight = 1;
    constraints.joint_constraints.push_back(jc);
    move_group.setPathConstraints(constraints);

    visual_tools.prompt("Move to starting pose");
    geometry_msgs::Pose start_pose;
    quaternionTFToMsg(tf::Quaternion(tf::Vector3(1, 0, 0), 0.0), start_pose.orientation);
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.30;
    move_group.setPoseTarget(start_pose);
    move_group.setPlanningTime(15.0);
    move_group.move();

    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    visual_tools.prompt("Walk through a square from starting pose");
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);
    geometry_msgs::Pose waypoint_pose = start_pose;
    waypoint_pose.position.z += 0.1;
    waypoints.push_back(waypoint_pose); // down
    waypoint_pose.position.y -= 0.1;
    waypoints.push_back(waypoint_pose); // right
    waypoint_pose.position.z -= 0.1;
    waypoint_pose.position.y += 0.1;
    waypoint_pose.position.x -= 0.1;
    waypoints.push_back(waypoint_pose); // up and left

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XSMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::XXSMALL);
    visual_tools.trigger();

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(1.0);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; //this value prevent to compute path
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, constraints);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    robot_trajectory::RobotTrajectory robotTrajectory(move_group.getCurrentState()->getRobotModel(), "youbot");
    robotTrajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(robotTrajectory);

    moveit_msgs::RobotTrajectory timed_robot_trajectory;
    robotTrajectory.getRobotTrajectoryMsg(timed_robot_trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;
    target_plan.trajectory_ = timed_robot_trajectory;
    move_group.execute(target_plan);
  }
  ros::shutdown();
  return 0;
}