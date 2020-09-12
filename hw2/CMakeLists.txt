cmake_minimum_required(VERSION 2.8.3)
project(hw_2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    rosbag
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    apriltag_ros
    apriltag
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)    ###


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   msg_hm1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   srv_hm1.srv
#)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   actionlib_msgs
#)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES homework1_test
  CATKIN_DEPENDS roscpp std_msgs 
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  DEPENDS Boost EIGEN3
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}  ###
)

   ###
     ###

#add_executable(node_b src/node_b.cpp)
#add_executable(node_a src/node_a.cpp)
#add_executable(node_c src/node_c.cpp)
add_executable(node_d src/node_d.cpp)
target_link_libraries(node_d ${catkin_LIBRARIES})
#target_link_libraries(node_b ${catkin_LIBRARIES})
#target_link_libraries(node_a ${catkin_LIBRARIES})
#target_link_libraries(node_c ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_homework1_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


