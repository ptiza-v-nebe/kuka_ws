PROJECT(youbot_driver)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

############## ROS Stuff ##############
#handle youbot_driver as ros package
#######################################
find_package(catkin REQUIRED COMPONENTS
  roscpp  
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include src/soem/include
  LIBRARIES YouBotDriver soem
)


############## cmake flags ##############
# PIC -> position independend code
#########################################
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_C_FLAGS_RELEASE "-fPIC -Wall -Wno-unused-but-set-variable")
SET(CMAKE_CXX_FLAGS_RELEASE "-fPIC -Wall -Wno-unused-but-set-variable")
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")
#SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/installed")


############## Find Boost ###############
#Search for boost on system
#########################################
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)


############## Create Docs ##############
#Use doxygen and latex for documentary
#########################################
#SET(DOXYFILE_LATEX "NO")
#SET(DOXYFILE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/doc)
#include(UseDoxygen)


############## Add soem #################
#Simple Open EtherCAT Master Library
#########################################
add_subdirectory(src/soem soem)


########### Main youbot files ###########
#Build apps
#########################################
#define source files
SET(YOUBOT_DRIVER_SRC
  src/EthercatMaster.cpp
  src/EthercatMasterWithThread.cpp
  src/EthercatMasterWithoutThread.cpp
  src/generic/Logger.cpp
  src/generic/ConfigFile.cpp
  src/generic/PidController.cpp
  src/generic-joint/JointData.cpp
  src/generic-joint/JointTrajectory.cpp
  src/YouBotJoint.cpp
  src/JointLimitMonitor.cpp
  src/YouBotBase.cpp
  src/YouBotManipulator.cpp
  src/YouBotGripper.cpp
  src/YouBotGripperBar.cpp
  src/YouBotGripperParameter.cpp
  src/DataTrace.cpp
  src/GripperDataTrace.cpp
  src/YouBotJointParameter.cpp
  src/YouBotJointParameterReadOnly.cpp
  src/YouBotJointParameterPasswordProtected.cpp
  src/JointTrajectoryController.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)

#define header files
SET(YOUBOT_DRIVER_HEADERS
  EthercatMaster.hpp
  EthercatMasterInterface.hpp
  EthercatMasterWithThread.hpp
  EthercatMasterWithoutThread.hpp
  YouBotGripper.hpp
  YouBotGripperBar.hpp
  YouBotGripperParameter.hpp
  YouBotBase.hpp
  YouBotManipulator.hpp
  YouBotJoint.hpp
  JointLimitMonitor.hpp
  YouBotJointParameterReadOnly.hpp
  YouBotJointParameterPasswordProtected.hpp
  YouBotJointParameter.hpp
  YouBotSlaveMailboxMsg.hpp
  YouBotSlaveMsg.hpp
  ProtocolDefinitions.hpp
  YouBotJointStorage.hpp
  DataTrace.hpp
  GripperDataTrace.hpp
  JointTrajectoryController.hpp
  base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
  base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
  base-kinematic/WheeledBaseKinematic.hpp
  generic/Exceptions.hpp
  generic/Logger.hpp
  generic/Time.hpp
  generic/Units.hpp
  generic/ConfigFile.hpp
  generic/PidController.hpp
  generic/dataobjectlockfree/DataObjectLockFree.hpp
  generic/dataobjectlockfree/target.hpp
  generic/dataobjectlockfree/os/oro_arch.h
  generic/dataobjectlockfree/os/oro_gcc/oro_arch.h
  generic/dataobjectlockfree/os/oro_i386/oro_arch.h
  generic/dataobjectlockfree/os/oro_msvc/oro_arch.h
  generic/dataobjectlockfree//os/oro_powerpc/oro_arch.h  
  generic/dataobjectlockfree/os/oro_powerpc/oro_atomic.h  
  generic/dataobjectlockfree/os/oro_powerpc/oro_system.h
  generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h
  generic-gripper/Gripper.hpp
  generic-gripper/GripperData.hpp
  generic-gripper/GripperParameter.hpp
  generic-joint/Joint.hpp
  generic-joint/JointData.hpp
  generic-joint/JointTrajectory.hpp
  generic-joint/JointParameter.hpp
  one-dof-gripper/OneDOFGripper.hpp
  one-dof-gripper/OneDOFGripperData.hpp
)

#define configs
SET(YOUBOT_DRIVER_CONFIG
  ${PROJECT_SOURCE_DIR}/config/youbot-ethercat.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-base.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-manipulator.cfg
)

#static link of boost and soem to YouBotDriver
ADD_LIBRARY(YouBotDriver ${YOUBOT_DRIVER_SRC})
TARGET_LINK_LIBRARIES(YouBotDriver boost_thread boost_date_time boost_filesystem boost_system boost_regex)
TARGET_LINK_LIBRARIES(YouBotDriver soem)
TARGET_INCLUDE_DIRECTORIES(YouBotDriver PUBLIC include include/generic)

#create new shared library but link static libraries
ADD_LIBRARY(YouBotDriverShared SHARED "")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES OUTPUT_NAME "YouBotDriver")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(YouBotDriverShared -Wl,-whole-archive YouBotDriver soem -Wl,-no-whole-archive)


########### Install driver ##############
#Install libYouBotDriver.so and headers
#########################################
MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES include/${HEADER} DESTINATION include/youbot/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

MACRO(INSTALL_CONFIG_WITH_DIRECTORY CONFIG_LIST)
  FOREACH(CONFIG ${${CONFIG_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${CONFIG})
    INSTALL(FILES ${CONFIG} DESTINATION config)
  ENDFOREACH(CONFIG)
ENDMACRO(INSTALL_CONFIG_WITH_DIRECTORY)

#copy header and config files
INSTALL_HEADERS_WITH_DIRECTORY(YOUBOT_DRIVER_HEADERS)
INSTALL_CONFIG_WITH_DIRECTORY(YOUBOT_DRIVER_CONFIG)
#copy library and executables
INSTALL(TARGETS YouBotDriverShared RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

