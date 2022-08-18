cmake_minimum_required(VERSION 3.4.1)

set(PROJ_NAME MPC)
project(${PROJ_NAME}) 

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O3")

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/*.hpp")
FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/*.cpp")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})

include_directories(${CMAKE_SOURCE_DIR}) 

set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)

include_directories(${EIGEN_INCLUDE_DIRS}) 