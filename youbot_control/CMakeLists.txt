cmake_minimum_required(VERSION 2.8.3)
project(youbot_control)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_manager
  hardware_interface
  roscpp
  youbot_driver
  combined_robot_hw
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS youbot_driver control_toolbox controller_manager hardware_interface roscpp combined_robot_hw
)

## preprocessor definitions
#add_compile_options(-std=c++11)
#ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR=${youbot_driver_CONFIG_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(youbot_hardware_interfaces 
    src/youbot_manipulator_interface.cpp
    src/youbot_base_interface.cpp
)
target_link_libraries(youbot_hardware_interfaces ${catkin_LIBRARIES})

add_executable(youbot_control_node
    src/youbot_control_node.cpp
)
target_link_libraries(youbot_control_node ${catkin_LIBRARIES})

add_custom_command(TARGET youbot_control_node POST_BUILD
    COMMAND sudo setcap cap_net_raw+ep $<TARGET_FILE:youbot_control_node>  
)