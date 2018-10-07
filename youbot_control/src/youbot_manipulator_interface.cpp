#include <youbot_control/youbot_manipulator_interface.hpp>

using namespace hardware_interface;
using namespace youbot;

namespace youbot_hardware_interface
{
YoubotManipulatorInterface::YoubotManipulatorInterface(){

}

YoubotManipulatorInterface::~YoubotManipulatorInterface(){

}

bool YoubotManipulatorInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    myYouBotManipulator = new YouBotManipulator("youbot-manipulator", "/usr/local/config");
    myYouBotManipulator->doJointCommutation();
    myYouBotManipulator->calibrateManipulator();
    //get joint names and num of joint
    robot_hw_nh.getParam("joints", joint_name);
    num_joints = joint_name.size();

    //n[0] = joint_name[0][10]; //arm_joint_4
    //ROS_INFO("n: %f", std::stod(n[0]));

    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    //joint_position_command.resize(num_joints);
    //joint_velocity_command.resize(num_joints);
    joint_effort_command.resize(num_joints);
 
    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        

        //Effort
        JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command[i]);
        effort_joint_interface.registerHandle(jointEffortHandle);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&effort_joint_interface);
    return true;
}

void YoubotManipulatorInterface::read(const ros::Time& time, const ros::Duration& period){
     JointSensedAngle sensedAngle; //angle, radian
     JointSensedVelocity sensedVelocity; //angularVelocity, radian_per_second
     JointSensedCurrent sensedCurrent; //current, ampere
     for(int i=0; i<num_joints; i++){
        myYouBotManipulator->getArmJoint(i+1).getData(sensedAngle);
        myYouBotManipulator->getArmJoint(i+1).getData(sensedVelocity);
        myYouBotManipulator->getArmJoint(i+1).getData(sensedCurrent);

        joint_position_state[i] = sensedAngle.angle.value();
        joint_velocity_state[i] = sensedVelocity.angularVelocity.value();
        joint_effort_state[i] = sensedCurrent.current.value();
     }
}

void YoubotManipulatorInterface::write(const ros::Time& time, const ros::Duration& period){
    JointCurrentSetpoint desiredCurrent;
    for(int i=0; i<num_joints; i++){
        desiredCurrent.current = joint_effort_command[i]*ampere;
        myYouBotManipulator->getArmJoint(i+1).setData(desiredCurrent);
    }
}
}
PLUGINLIB_EXPORT_CLASS(youbot_hardware_interface::YoubotManipulatorInterface, hardware_interface::RobotHW)