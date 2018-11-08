#include <youbot_control_combi/youbot_base_interface.hpp>

using namespace hardware_interface;
using namespace youbot;

namespace youbot_hardware_interface
{

YoubotBaseInterface::YoubotBaseInterface(){

}

YoubotBaseInterface::~YoubotBaseInterface(){

}

bool YoubotBaseInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    //init base
    myYouBotBase = new YouBotBase("youbot-base", "/usr/local/config");
	myYouBotBase->doJointCommutation();

    //get joint names and num of joint
    robot_hw_nh.getParam("joints", joint_name);
    num_joints = joint_name.size();

    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    //joint_position_command.resize(num_joints);
    joint_velocity_command.resize(num_joints);
    //joint_effort_command.resize(num_joints);
 
    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        

        //Velocity
        JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command[i]);
        velocity_joint_interface.registerHandle(jointVelocityHandle);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
    return true;
}

void YoubotBaseInterface::read(const ros::Time& time, const ros::Duration& period){
        myYouBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        x = longitudinalPosition.value();
        y = transversalPosition.value();
        theta = orientation.value();

        myYouBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        vx = longitudinalVelocity.value();
        vy = transversalVelocity.value();
        vtheta = angularVelocity.value();

        joint_position_state[0] = x;
        joint_position_state[1] = y;
        joint_position_state[2] = theta;

        joint_velocity_state[0] = vx;
        joint_velocity_state[1] = vy;
        joint_velocity_state[2] = vtheta;
        //ROS_INFO("Perceived odometric values (x,y,theta, vx,vy,vtheta): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);
}

void YoubotBaseInterface::write(const ros::Time& time, const ros::Duration& period){
    quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

    float xWC = joint_velocity_command[0];
    float yWC = joint_velocity_command[1];
    float thetaWC = joint_velocity_command[2];

    float xRC = cos(theta)*xWC + sin(theta)*yWC;
    float yRC = -sin(theta)*xWC + cos(theta)*yWC;

    longitudinalVelocity = xRC * meter_per_second;
	transversalVelocity = yRC * meter_per_second;
	angularVelocity = thetaWC * radian_per_second;
	myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}
}
PLUGINLIB_EXPORT_CLASS(youbot_hardware_interface::YoubotBaseInterface, hardware_interface::RobotHW)