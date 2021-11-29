cmake_minimum_required(VERSION 2.8.6)
project(egmri_controller_pkg)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    sensor_msgs
    pluginlib
    roscpp
    roslib
    rospy
    control_toolbox
    realtime_tools
#    orocos_kdl
    geometry_msgs
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(orocos_kdl)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
    ControllerParams.msg
    DataRequest.msg
    DataType.msg
    PositionCommand.msg
    SampleResult.msg
    TfActionCommand.msg
    TfObsData.msg
    TfParams.msg
    TrialCommand.msg
    Custom.msg
)

#uncomment if you have defined messages
generate_messages(DEPENDENCIES std_msgs)

#Include protobuf
include_directories($ENV{EGMRI_ROOT_DIR}/build/egmri)

#Include boost
find_package(Boost 1.46.0 COMPONENTS)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

# Enable C++11 if available
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly, not enabling C++11.")
endif()

include_directories(include)

set(DDP_FILES src/robotplugin.cpp
              src/yumiplugin.cpp
              src/sample.cpp
              src/sensor.cpp
              src/tfcontroller.cpp
              src/controller.cpp
              src/positioncontroller.cpp
              src/trialcontroller.cpp
              src/encodersensor.cpp
              src/encoderfilter.cpp
              src/util.cpp)


#uncomment for test executable
option(BUILD_TEST "Build the test executables" OFF)

add_library(egmri_controller_lib ${DDP_FILES})
add_dependencies(egmri_controller_lib  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(BUILD_TEST)
    add_executable(main src/main.cpp)
    add_executable(kinematic_baseline src/kinematic_baseline.cpp)
    add_executable(controller_switcher src/controller_switcher.cpp)
    add_executable(pointcloud_solver src/pointcloud_solver.cpp
                                     src/keypoint_detector.cpp)
    add_executable(point_head src/point_head.cpp)
    add_executable(torso src/torso.cpp)
endif()

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
