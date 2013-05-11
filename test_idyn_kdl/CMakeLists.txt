
# Copyright (C) 2010 RobotCub Consortium
# Author: Matteo Fumagalli, Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.8)


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB)
find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
 
SET(PROJECTNAME misc_src)

PROJECT(${PROJECTNAME})

#FILE(GLOB folder_source force_test.cpp ../lib/chain_conversion.cpp ../lin/inertia_common.cpp)
#FILE(GLOB folder_header ../lib/chain_conversion.h ../lib/inertia_common.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

#ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

#TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn ${YARP_LIBRARIES} skinDynLib ${Orocos-KDL_LIBRARIES})

#INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_executable(force_test_icub_arm force_test_icub_arm.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_no_sensor force_test_no_sensor.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_3links force_test_3links.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_5links force_test_5links.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
add_executable(force_test_7links force_test_7links.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_execution_time force_test_execution_time.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)


target_link_libraries(force_test_icub_arm ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_no_sensor ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_3links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_5links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
target_link_libraries(force_test_7links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_execution_time ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})

