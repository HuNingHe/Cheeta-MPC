cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
project(centroid_mpc_test)

include_directories(../../../Controllers)
add_executable(centroid_mpc_test ../src/CentroidalMPC.cpp CentoidMPCTest.cpp)

target_link_libraries(centroid_mpc_test casadi)