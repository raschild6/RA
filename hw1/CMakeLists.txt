cmake_minimum_required(VERSION 2.8.3)
project(hw1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  apriltag_ros
  apriltag
  tf
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)


find_package(Boost REQUIRED COMPONENTS system)

find_package( PCL REQUIRED )    ###


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
generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES homework1_test
  CATKIN_DEPENDS roscpp std_msgs message_runtime actionlib
  DEPENDS Boost
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}   ###
)

link_directories(${PCL_LIBRARY_DIRS})   ###
add_definitions(${PCL_DEFINITIONS})     ###

add_executable(node_hw1_task_1 src/node_hw1_task_1.cpp)
add_executable(node_hw1_task_2 src/node_hw1_task_2.cpp)
add_executable(node_hw1_task_2_visual src/node_hw1_task_2_visual.cpp)


target_link_libraries(node_hw1_task_2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(node_hw1_task_2_visual ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(node_hw1_task_1 ${catkin_LIBRARIES})


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


