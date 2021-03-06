#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <youbot_control/youbot_interface.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "youbot_control_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    
    youbot_hardware_interface::YoubotInterface hw;
    bool init_success = hw.init(nh,nh);

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Duration period(1.0/200);

    ROS_INFO("youbot_control started");
    while(ros::ok()){
        hw.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
return 0;
}
