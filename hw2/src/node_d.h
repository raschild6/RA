#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <gazebo_msgs/SetModelState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>