cmake_minimum_required(VERSION 2.8)
project(youbotUnitTest)

#SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}")
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

FIND_PACKAGE(CppUnit REQUIRED)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
FIND_PACKAGE(YoubotDriver REQUIRED)
MESSAGE("${YOUBOT_DRIVER_CONFIG_DIR}/")
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${YOUBOT_DRIVER_CONFIG_DIR}/")

SET(YOUBOT_TEST_INCLUDE
  include/YouBotArmTest.hpp  
  include/YouBotArmTestWithoutThread.hpp  
  include/YouBotBaseKinematicsTest.hpp  
  include/YouBotBaseTest.hpp  
  include/YouBotBaseTestWithoutThread.hpp  
  include/YouBotGripperTest.hpp
)

SET(YOUBOT_TEST_SRC
  src/TestSuite.cpp
  src/YouBotBaseTest.cpp
  src/YouBotBaseTestWithoutThread.cpp
  src/YouBotBaseKinematicsTest.cpp
  src/YouBotArmTest.cpp
  src/YouBotArmTestWithoutThread.cpp
  src/YouBotGripperTest.cpp
)

ADD_EXECUTABLE(base_arm_gripper_test ${YOUBOT_TEST_SRC})
target_link_libraries(base_arm_gripper_test ${YOUBOT_DRIVER_LIBRARIES} ${CPPUNIT_LIBRARIES} boost_thread boost_system)
target_include_directories(base_arm_gripper_test PRIVATE include ${YOUBOT_DRIVER_INCLUDE_DIR})

#INSTALL(TARGETS base_arm_gripper_test RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

