#include <youbot_control/youbot_interface.hpp>

using namespace hardware_interface;
using namespace youbot;

namespace youbot_hardware_interface
{

YoubotInterface::YoubotInterface(){

}

YoubotInterface::~YoubotInterface(){

}

bool YoubotInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    //initialise manipulator
    myYouBotManipulator = new YouBotManipulator("youbot-manipulator", "/usr/local/config");
    myYouBotManipulator->doJointCommutation();
    myYouBotManipulator->calibrateManipulator();

    //initialise base
    myYouBotBase = new YouBotBase("youbot-base", "/usr/local/config");
	myYouBotBase->doJointCommutation();

    //get joint names and num of joint
    robot_hw_nh.getParam("/youbot/hardware_interface/joints", joint_name);
    num_joints = joint_name.size();

    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
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

void YoubotInterface::read(const ros::Time& time, const ros::Duration& period){
    //read states from manipulator
    JointSensedAngle sensedAngle; //angle, radian
    JointSensedVelocity sensedVelocity; //angularVelocity, radian_per_second
    JointSensedCurrent sensedCurrent; //current, ampere
    for(int i=0; i<5; i++){
        myYouBotManipulator->getArmJoint(i+1).getData(sensedAngle);
        myYouBotManipulator->getArmJoint(i+1).getData(sensedVelocity);
        myYouBotManipulator->getArmJoint(i+1).getData(sensedCurrent);

        joint_position_state[i] = sensedAngle.angle.value();
        joint_velocity_state[i] = sensedVelocity.angularVelocity.value();
        joint_effort_state[i] = sensedCurrent.current.value();
    }
    
     //read states from base
    myYouBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
    x = longitudinalPosition.value();
    y = transversalPosition.value();
    theta = orientation.value();

    myYouBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
    vx = longitudinalVelocity.value();
    vy = transversalVelocity.value();
    vtheta = angularVelocity.value();

    joint_position_state[5] = x;
    joint_position_state[6] = y;
    joint_position_state[7] = theta;

    //joint_velocity_state[5] = vx;
    //joint_velocity_state[6] = vy;
    //joint_velocity_state[7] = vtheta;

    joint_effort_state[5] = vx;
    joint_effort_state[6] = vy;
    joint_effort_state[7] = vtheta;
}

void YoubotInterface::write(const ros::Time& time, const ros::Duration& period){
    //write command to manipulator
    JointCurrentSetpoint desiredCurrent;
    for(int i=0; i<5; i++){
        desiredCurrent.current = joint_effort_command[i]*ampere;
        myYouBotManipulator->getArmJoint(i+1).setData(desiredCurrent);
    }

    //write command to base
    quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

    float xWC = joint_effort_command[5];
    float yWC = joint_effort_command[6];
    float thetaWC = joint_effort_command[7];

    float xRC = cos(theta)*xWC + sin(theta)*yWC;
    float yRC = -sin(theta)*xWC + cos(theta)*yWC;

    longitudinalVelocity = xRC * meter_per_second;
	transversalVelocity = yRC * meter_per_second;
	angularVelocity = thetaWC * radian_per_second;
	myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}

}