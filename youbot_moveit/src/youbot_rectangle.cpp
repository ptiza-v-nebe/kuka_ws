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

#include "tf2/LinearMath/Vector3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <math.h>

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

  move_group.clearPathConstraints();

  visual_tools.prompt("move to pose that initially satisfy joint constraints");
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> initial_positions;
  current_state->copyJointGroupPositions(joint_model_group, initial_positions);
  initial_positions[3] = 3.0;
  initial_positions[4] = 1.3;
  initial_positions[5] = -1.3;
  move_group.setJointValueTarget(initial_positions);
  move_group.move();

  moveit_msgs::JointConstraint jc;
  moveit_msgs::Constraints constraints;
  jc.joint_name = "arm_joint_1";
  jc.position = 3.0;
  jc.tolerance_above = 0.1;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_2";
  jc.position = 1.3;
  jc.tolerance_above = 3.14;
  jc.tolerance_below = 0.5;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);
  jc.joint_name = "arm_joint_3";
  jc.position = -1.3;
  jc.tolerance_above = 1.57;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  constraints.joint_constraints.push_back(jc);

  move_group.setPathConstraints(constraints);

  while (ros::ok())
  {

    visual_tools.prompt("Move to starting pose");
    geometry_msgs::Pose start_pose;
    start_pose.orientation = tf2::toMsg( tf2::Quaternion(tf2::Vector3(1, 0, 0), 0.0) );
    
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.32;
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

    /*tf2::Transform transfrm(tf2::Quaternion(tf2::Vector3(1,0,0), M_PI/4.0), tf2::Vector3(start_pose.position.x,start_pose.position.y,start_pose.position.z));
  
    std::vector<tf2::Vector3> points_unrotated;
    points_unrotated.push_back(tf2::Vector3(0, 0.05, 0.05));
    points_unrotated.push_back(tf2::Vector3(0,-0.05, 0.05));
    points_unrotated.push_back(tf2::Vector3(0,-0.05,-0.05));
    points_unrotated.push_back(tf2::Vector3(0, 0.05,-0.05));
    points_unrotated.push_back(tf2::Vector3(0, 0.05, 0.05));

    std::vector<tf2::Vector3> points_rotated;
    for (int i =0; i<points_unrotated.size(); i++){
      points_rotated.push_back(transfrm * points_unrotated[i]);
    }
    
    std::vector<geometry_msgs::Pose> waypoints;
    for (int i = 0; i < points_rotated.size(); i++){
      geometry_msgs::Point pt;
      tf2::toMsg(points_rotated[i], pt);
      geometry_msgs::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1;
      waypoints.push_back(pose);
    }*/
    tf2::Transform transfrm(tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 4.0), tf2::Vector3(start_pose.position.x, start_pose.position.y, start_pose.position.z));
    std::vector<tf2::Transform> poses_unrotated_tf;
    /*poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1,0,0), 0), tf2::Vector3(0, 0.05, 0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1,0,0), M_PI_4), tf2::Vector3(0,-0.05, 0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1,0,1), M_PI_4), tf2::Vector3(0,-0.05,-0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1,0,0), M_PI_4), tf2::Vector3(0, 0.05,-0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1,1,1), M_PI_2), tf2::Vector3(0, 0.05, 0.05)));
    */

    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0), tf2::Vector3(0, 0.05, 0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0), tf2::Vector3(0, -0.05, 0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0), tf2::Vector3(0, -0.05, -0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0), tf2::Vector3(0, 0.05, -0.05)));
    poses_unrotated_tf.push_back(tf2::Transform(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0), tf2::Vector3(0, 0.05, 0.05)));

    std::vector<tf2::Transform> poses_rotated_tf;
    for (int i = 0; i < poses_unrotated_tf.size(); i++)
    {
      poses_rotated_tf.push_back(transfrm * poses_unrotated_tf[i]);
    }

    std::vector<geometry_msgs::Pose> waypoints;
    for (int i = 0; i < poses_unrotated_tf.size(); i++)
    {
      geometry_msgs::Pose pose;
      waypoints.push_back(tf2::toMsg(poses_rotated_tf[i], pose));
    }

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
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory /*, constraints*/);
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