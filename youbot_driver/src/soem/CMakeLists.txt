#### SOEM ####
PROJECT(youBot_datatrace)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

SET(SOEM_HEADERS
    ethercatbase.h
    ethercatcoe.h
    ethercatconfig.h
    ethercatconfiglist.h
    ethercatdc.h
    ethercatfoe.h
    ethercatmain.h
    ethercatprint.h
    ethercatsoe.h
    ethercattype.h
    nicdrv.h
)

SET(SOEM_SRC
    src/ethercatbase.c 
    src/ethercatcoe.c 
    src/ethercatconfig.c 
    src/ethercatfoe.c 
    src/ethercatsoe.c 
    src/ethercatmain.c 
    src/ethercatprint.c 
    src/nicdrv.c 
    src/ethercatdc.c
)
SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/installed")

ADD_LIBRARY(soem STATIC ${SOEM_SRC})
TARGET_INCLUDE_DIRECTORIES(soem PUBLIC include)

MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES include/${HEADER} DESTINATION include/youbot/soem/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

INSTALL_HEADERS_WITH_DIRECTORY(SOEM_HEADERS)
