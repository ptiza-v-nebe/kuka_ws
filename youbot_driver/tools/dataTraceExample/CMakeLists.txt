PROJECT(dataTraceExample)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

## check required prerequisites, build parameters and paths, preprocessor definitions
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


## Find Boost and YoubotDriver
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)
FIND_PACKAGE(YoubotDriver REQUIRED)
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${YOUBOT_DRIVER_CONFIG_DIR}/")

## Compile and link
ADD_EXECUTABLE(youBot_dataTrace
  src/main.cpp
)

TARGET_LINK_LIBRARIES(youBot_dataTrace 
	${YOUBOT_DRIVER_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
)

TARGET_INCLUDE_DIRECTORIES(youBot_dataTrace PRIVATE
	${YOUBOT_DRIVER_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
)

#INSTALL(TARGETS youBot_dataTrace RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
