# Standard setup cmake_minimum_required (VERSION 2.6)
cmake_minimum_required (VERSION 2.6)
project (MotorTrial)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

set(includeDirs ${includeDirs} src)
set(includeDirs ${includeDirs} ../ExternalSoftware/serialComm)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/motorTest.cpp)
set(SRC ${SRC} ../ExternalSoftware/serialComm/rs232.c)

set(H ${H} src/motorTest.h)
set(H ${H} ../ExternalSoftware/serialComm/rs232.h)

include_directories (${includeDirs})
add_executable (MotorTrial ${SRC} ${H})
target_link_libraries (MotorTrial ${LIBS} -ldl)
install (TARGETS MotorTrial DESTINATION ${MotorTrial_BINARY_DIR} )
