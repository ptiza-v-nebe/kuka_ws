cmake_minimum_required(VERSION 2.8.3)
project(youbot_moveit)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  #interactive_markers
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  roscpp
  rospy
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    #interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    #interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#add_library(inter_utils
#  src/interactivity/interactive_robot.cpp
#  src/interactivity/imarker.cpp
#  src/interactivity/pose_string.cpp
#)

add_executable(youbot_joint_constraint src/youbot_joint_constraint.cpp)
target_link_libraries(youbot_joint_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(youbot_position_constraint src/youbot_position_constraint.cpp)
target_link_libraries(youbot_position_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(youbot_orientation_constraint src/youbot_orientation_constraint.cpp)
target_link_libraries(youbot_orientation_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(youbot_cartesian_path_constraint src/youbot_cartesian_path_constraint.cpp)
target_link_libraries(youbot_cartesian_path_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(youbot_rectangle src/youbot_rectangle.cpp)
target_link_libraries(youbot_rectangle ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(youbot_interactive_robot src/youbot_interactive_robot.cpp)
#target_link_libraries(youbot_interactive_robot inter_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(interactivity_tutorial src/interactivity_tutorial.cpp)
#target_link_libraries(interactivity_tutorial inter_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
