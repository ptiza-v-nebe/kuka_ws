# - Try to find YOUBOT_DRIVER
# Once done this will define
#
#  YOUBOT_DRIVER_FOUND - if YOUBOT_DRIVER was found
#  YOUBOT_DRIVER_INCLUDE_DIR - the YOUBOT_DRIVER include directory
#  YOUBOT_DRIVER_LIBRARIES - YOUBOT_DRIVER libraries
#  YOUBOT_DRIVER_CONFIG_DIR - YOUBOT_DRIVER configuration directory
#
#
# The configuration directory YOUBOT_DRIVER_CONFIG_DIR will always be
# set, either to a directory containing the "youbout-ethercat.cfg" file,
# or if no such file is found, to the local folder ".". In the last
# case a warning message is  issued.
#

FIND_PATH(YOUBOT_DRIVER_INCLUDE_DIR NAMES YouBotBase.hpp
  PATHS
  #$ENV{YOUBOTDIR}/include/youbot
  #ENV CPATH
  /usr/local/include/youbot
  NO_DEFAULT_PATH
)
MESSAGE("YOUBOT_DRIVER_INCLUDE_DIR: ${YOUBOT_DRIVER_INCLUDE_DIR}")

FIND_LIBRARY(YOUBOT_DRIVER_LIBRARIES NAMES "YouBotDriver" "soem"
  PATHS
  #$ENV{YOUBOTDIR}/lib 
  #ENV LD_LIBRARY_PATH
  #ENV LIBRARY_PATH
  /usr/local/lib
  NO_DEFAULT_PATH
)
MESSAGE("YOUBOT_DRIVER_LIBRARIES: ${YOUBOT_DRIVER_LIBRARIES}")

FIND_PATH(YOUBOT_DRIVER_CONFIG_DIR NAMES youbot-ethercat.cfg
  PATHS
  #$ENV{YOUBOTDIR}/config/
  /usr/local/config
  NO_DEFAULT_PATH
)
MESSAGE("YOUBOT_DRIVER_CONFIG_DIR: ${YOUBOT_DRIVER_CONFIG_DIR}")

SET(YOUBOT_DRIVER_INCLUDE_DIR ${YOUBOT_DRIVER_INCLUDE_DIR} ${YOUBOT_DRIVER_INCLUDE_DIR}/soem)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set YOUBOT_DRIVER_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(YOUBOT_DRIVER  DEFAULT_MSG
                                  YOUBOT_DRIVER_INCLUDE_DIR YOUBOT_DRIVER_LIBRARIES)

# show the YOUBOT_DRIVER_INCLUDE_DIR, YOUBOT_DRIVER_LIBRARIES and YOUBOT_DRIVER_CONFIG_DIR variables only in the advanced view
IF(YOUBOT_DRIVER_FOUND)
  MARK_AS_ADVANCED(YOUBOT_DRIVER_INCLUDE_DIR YOUBOT_DRIVER_LIBRARIES_DIR YOUBOT_DRIVER_CONFIG_DIR)
ENDIF(YOUBOT_DRIVER_FOUND)

# show warning that no configuration directory was found; given that YOUBOT_DRIVER itself was found
IF (YOUBOT_DRIVER_FOUND AND NOT YOUBOT_DRIVER_CONFIG_DIR)
  SET(YOUBOT_DRIVER_CONFIG_DIR .)
  IF (NOT YOUBOT_DRIVER_FIND_QUIETLY)
    MESSAGE(STATUS "Warning: youbot_driver configuration directory not found, using default folder \".\"")
  ENDIF (NOT YOUBOT_DRIVER_FIND_QUIETLY)
ENDIF (YOUBOT_DRIVER_FOUND AND NOT YOUBOT_DRIVER_CONFIG_DIR)

