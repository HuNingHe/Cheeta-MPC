cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

set(CUSTOM_RBDL_PATH "/home/hun/rbdl-orb/install/usr/local")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
find_package(RBDL REQUIRED)   # required
find_package(Eigen3 REQUIRED) # required
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

#if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic -std=c++11 -fext-numeric-literals")
#endif()
add_subdirectory(third-party)
link_directories(third-party/qpSWIFT/bin/arc third-party/qpSWIFT/bin/lib)

file(GLOB ESTIMATOR_SRCS "Estimator/src/*.cpp")
file(GLOB FSM_STATES_SRCS "FSM_States/src/*.cpp")
file(GLOB GAIT_PLANNER_SRCS "GaitPlanner/src/*.cpp")
file(GLOB KINO_DYN_SRCS "Kino_Dynamics/src/*.cpp")
file(GLOB WEBOTS_INTERFACE "WebotsDevices/*.cpp")
file(GLOB_RECURSE CONTROLLERS_SRCS "Controllers/*.cpp" "Controllers/*.h" "Controllers/*.hpp")

include_directories(Controllers)
include_directories(Estimator/include)
include_directories(FSM_States/include)
include_directories(GaitPlanner/include)
include_directories(Kino_Dynamics/include)
include_directories(third-party/Goldfarb_Optimizer)
include_directories(third-party/qpOASES/include)
include_directories(third-party/qpSWIFT/include)

include_directories(Utilities)
include_directories(WebotsDevices)
include_directories (${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

set(SOURCES
    main.cpp
    Utilities/ControlParameters.cpp
    Utilities/save_data_to_file.cpp
    ${WEBOTS_INTERFACE}
    ${KINO_DYN_SRCS}
    ${GAIT_PLANNER_SRCS}
    ${FSM_STATES_SRCS}
    ${ESTIMATOR_SRCS}
    ${CONTROLLERS_SRCS}
    )

set(WEBOTS_HOME "/usr/local/webots")
include_directories(${WEBOTS_HOME}/include/controller/cpp)

link_directories(${WEBOTS_HOME}/lib/controller)
set (WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT}
        ${WEBOTS_LIBRARIES}
        ${RBDL_LIBRARY}
        qpOASES
        Goldfarb_Optimizer
        rt
        qpSWIFT
        casadi
        )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
