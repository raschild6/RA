// ROS headers
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
   

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_a");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //Get model for robot we want to move
    //robot_model_loader::RobotModelLoader robot_model_loader("ur5/robot_description");

    //Group we want to move
    static const std::string PLANNING_GROUP = "manipulator";
    static const std::string ROBOT = "ur5/robot_description";

    //Interface for moving group
    moveit::planning_interface::MoveGroupInterface move_group(moveit::planning_interface::MoveGroupInterface::Options(PLANNING_GROUP, ROBOT));
    ROS_INFO("Created move group");
    //Planning scene interface can be used to add and remove collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    // We can print the name of the reference frame for this robot.
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());


    // We can get a list of all the groups in the robot:
    ROS_INFO("Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
        std::ostream_iterator<std::string>(std::cout, ", "));



    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);


    // Now, we call the planner to compute the plan 
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);



    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal.

    move_group.move();
/*
   

  const std::string PLANNING_GROUP = "manipulator";
  robot_model_loader::RobotModelLoader robot_model_loader("ur5/robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
  // that maintains the state of the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "rest_up_left");

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }



  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }



  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  //Set the state in the planning scene to the final state of the last plan 
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

 
 */
 
    spinner.stop();
   
    return EXIT_SUCCESS;
}