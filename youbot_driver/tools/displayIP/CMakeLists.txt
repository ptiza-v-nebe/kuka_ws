cmake_minimum_required(VERSION 2.8)
PROJECT(DISPLAYIP)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

ADD_EXECUTABLE(displayIpAddress src/DisplayIpAddress.cpp)

target_link_libraries(displayIpAddress stdc++)

INSTALL(TARGETS displayIpAddress RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

