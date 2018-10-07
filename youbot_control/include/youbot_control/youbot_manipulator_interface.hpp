#ifndef YOUBOT_MANIP_INTERFACE_HPP
#define YOUBOT_MANIP_INTERFACE_HPP

#include <iostream>
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>

using namespace std;
using namespace hardware_interface;
using namespace youbot;

namespace youbot_hardware_interface
{
class YoubotManipulatorInterface: public hardware_interface::RobotHW
{
public:
    YoubotManipulatorInterface();
    ~YoubotManipulatorInterface();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

protected:
    ros::NodeHandle nh_;
    
    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    //hardware_interface::PositionJointInterface position_joint_interface;
    //hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    int num_joints;
    vector<string> joint_name;
    vector<string> n;

    //actual states
    vector<double> joint_position_state;
    vector<double> joint_velocity_state;
    vector<double> joint_effort_state;

    //given setpoints
    vector<double> joint_position_command;
    vector<double> joint_velocity_command;
    vector<double> joint_effort_command;

    //Youbot
    YouBotManipulator* myYouBotManipulator;
};
}
#endif
