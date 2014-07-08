# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(jaco)

set(MESSAGE_DEPENDENCIES
geometry_msgs
std_msgs
actionlib_msgs
trajectory_msgs
control_msgs
)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp urdf actionlib message_generation ${MESSAGE_DEPENDENCIES})


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#message(STATUS ${Boost_LIBRARIES})

INCLUDE(FindPkgConfig)

### Finding glib-2.0
find_package(glib REQUIRED)
IF(glib-2.0_FOUND)
        MESSAGE(STATUS "Looking for glib-2.0 - found")
ELSE(glib-2.0_FOUND)
        MESSAGE(FATAL_ERROR "Looking for glib-2.0 - not found")
ENDIF(glib-2.0_FOUND)

### Finding mono-2.0
find_package(mono REQUIRED)
IF(mono-2.0_FOUND)
        MESSAGE(STATUS "Looking for mono-2.0 - found")
ELSE(mono-2.0_FOUND)
        MESSAGE(FATAL_ERROR "Looking for mono-2.0 - not found")
ENDIF(mono-2.0_FOUND)

#builds the csharp wrapper necessary to use jaco
execute_process(COMMAND xbuild ${PROJECT_SOURCE_DIR}/../CSharpWrapper/CSharpWrapper/CSharpWrapper.csproj
                  RESULT_VARIABLE Result
                  OUTPUT_VARIABLE Output
                  ERROR_VARIABLE Error)
if(Result EQUAL 0)
    message(STATUS "CSharpWrapper build success")
else()
    message(FATAL_ERROR "CSharpwrapper build Result - ${Result}\nOutput - ${Output}\nError - Error")
endif()

set(CSharpWrapper_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../CSharpWrapper/CSharpWrapper/bin/Debug)


add_message_files(
  FILES JacoPose.msg JacoPoseStamped.msg JacoPoseTrajectory.msg Point.msg
)

add_action_files(
  FILES CartesianMovement.action FingerMovement.action
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${mono-2.0_INCLUDE_DIRS} ${glib-2.0_INCLUDE_DIRS} ${CSharpWrapper_INCLUDE_DIR})

set(Include_Libs optimized ${mono-2.0_INCLUDE_LIBS} ${glib-2.0_INCLUDE_LIBS})

add_executable(jaco src/abstract_jaco.cpp  src/jaco_node.cpp src/jaco.cpp src/jaco_joint_publisher.cpp src/jaco_joystick_publisher src/jaco_action_controller.cpp src/gripper_controller.cpp)

#If you have a package which builds messages and/or services as well as executables that use them, you #need to create an explicit dependency on the automatically-generated message target so that they are #built in the correct order:
add_dependencies(jaco ${PROJECT_NAME}_generate_messages_cpp)

find_package(Boost REQUIRED COMPONENTS thread)

target_link_libraries(jaco ${Boost_LIBRARIES})
target_link_libraries(jaco ${catkin_LIBRARIES})
target_link_libraries(jaco ${Include_Libs})

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES ${MESSAGE_DEPENDENCIES}
)



# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package

catkin_package(
    #DEPENDS 
    CATKIN_DEPENDS message_runtime urdf actionlib ${MESSAGE_DEPENDENCIES}
    INCLUDE_DIRS include
    #LIBRARIES # TODO
)
