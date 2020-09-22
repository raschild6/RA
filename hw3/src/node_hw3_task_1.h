#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <numeric>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>

// Laser
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/ModelStates.h>

// Twist
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Feedback
#include <move_base_msgs/MoveBaseActionFeedback.h>

//Odometry
#include <nav_msgs/Odometry.h>

// Wrapper of planner
//#include <costmap_2d/costmap_2d_ros.h>
//#include <dwa_local_planner/dwa_planner_ros.h>
//#include <nav_core/base_global_planner.h>
//#include <nav_core/base_local_planner.h>
//#include <nav_core/recovery_behavior.h>




bool saveFatalCollision();      // need declaration first cause there is an intertwining of calls