cmake_minimum_required(VERSION 2.8.3)
project(homework1_test)

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
 add_message_files(
   FILES
   msg_hm1.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   srv_hm1.srv
)


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

add_executable(node_a src/node_a.cpp)
add_executable(node_b src/node_b.cpp)
add_executable(node_c src/node_c.cpp)
add_executable (correspondence_grouping src/correspondence_grouping.cpp)
add_executable(passthrough_test src/passthrough_test.cpp)

add_dependencies(node_a homework1_test_generate_messages_cpp)
add_dependencies(node_b homework1_test_generate_messages_cpp)
add_dependencies(node_c homework1_test_generate_messages_cpp)

target_link_libraries(node_a ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(node_b ${catkin_LIBRARIES})
target_link_libraries(node_c ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(passthrough_test ${PCL_LIBRARIES})
target_link_libraries(correspondence_grouping ${PCL_LIBRARIES} ${catkin_LIBRARIES})


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


