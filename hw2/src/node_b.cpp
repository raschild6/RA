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

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

 // Std C++ headers
   #include <string>
   #include <vector>
   #include <map>
   
   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "node_b");
        ros::NodeHandle nh;
     ros::AsyncSpinner spinner(1);
     spinner.start();
   

     /*if ( argc < 9 )
     {
       ROS_INFO(" ");
       ROS_INFO("\tUsage:");
       ROS_INFO(" ");
       ROS_INFO("\trosrun tiago_moveit_tutorial plan_arm_torso_fk torso_lift arm_1 arm_2 arm_3 arm_4 arm_5 arm_6 arm_7");
       ROS_INFO(" ");
       ROS_INFO("\twhere the list of arguments are the target values for the given joints");
       ROS_INFO(" ");
       return EXIT_FAILURE;
     }
   */
     std::map<std::string, double> target_position;
   
     target_position["shoulder_pan_joint"] = 0.0;
     target_position["shoulder_lift_joint"] = -1.5707;
     target_position["elbow_joint"] = 0.0;
     target_position["wrist_1_joint"] = -1.5707;
     target_position["wrist_2_joint"] = 0.0;
     target_position["wrist_3_joint"] = 0.0;
   

  //Group to move
  const std::string PLANNING_GROUP = "manipulator";
  
  //Robot model
  robot_model_loader::RobotModelLoader robot_model_loader("ur5/robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("CREATED MODEL");
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  /*const std::vector< std::string > & links = joint_model_group->getLinkModelNames();
     for (unsigned int k = 0; k < links.size(); ++k) {
         ROS_INFO("link: %s", links[k].c_str());
       }*/
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  //print some info
  //kinematic_model->printModelInfo(std::cout);
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  /*const std::vector< std::string > & groups = kinematic_model->getJointModelGroupNames();
    for (unsigned int i = 0; i < groups.size(); ++i) {
        ROS_INFO("group: %s", groups[i].c_str());
    }
*/
  //Scene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "rest_up_left");

  //Planner
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

if (!nh.getParam("ur5/move_group/planning_plugin", planner_plugin_name))
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
  if (!planner_instance->initialize(kinematic_model, nh.getNamespace()))
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


  //Pose
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;  
  pose.pose.orientation.w = 1.0;

  //Tolerance
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);



  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);


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
  //ROS_INFO("MESSAGE: ", response.c_Str());

  kinematic_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*kinematic_state.get());


  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


     moveit::planning_interface::MoveGroupInterface group_manipulator(PLANNING_GROUP);
//geometry_msgs::Pose move_target = srv.response.pose;
//group_manipulator.setPoseReferenceFrame(base_frame);
//group_manipulator.setPoseTarget(pose); 
group_manipulator.move();






  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");


  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");





     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

     std::vector<std::string> manipulator_joint_names;
     //select group of joints
     //moveit::planning_interface::MoveGroupInterface group_manipulator(PLANNING_GROUP);
     //choose your preferred planner
     group_manipulator.setPlannerId("SBLkConfigDefault");
   
     manipulator_joint_names = group_manipulator.getJoints();
   
     group_manipulator.setStartStateToCurrentState();
     group_manipulator.setMaxVelocityScalingFactor(1.0);
   
     for (unsigned int i = 0; i < manipulator_joint_names.size(); ++i)
       if ( target_position.count(manipulator_joint_names[i]) > 0 )
       {
         ROS_INFO_STREAM("\t" << manipulator_joint_names[i] << " goal position: " << target_position[manipulator_joint_names[i]]);
         group_manipulator.setJointValueTarget(manipulator_joint_names[i], target_position[manipulator_joint_names[i]]);
       }
   
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
     group_manipulator.setPlanningTime(5.0);
	 moveit::planning_interface::MoveItErrorCode success = group_manipulator.plan(my_plan);
   
     if ( success != moveit_msgs::MoveItErrorCodes::SUCCESS )
       throw std::runtime_error("No plan found");
   
     ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
   
     // Execute the plan
     ros::Time start = ros::Time::now();
   
     group_manipulator.move();
   
     ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
   
     spinner.stop();
   
     return EXIT_SUCCESS;
  }