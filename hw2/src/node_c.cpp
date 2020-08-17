#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_c");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // SETUP
  // --------------

  // Group to move
  static const std::string PLANNING_GROUP = "manipulator";

  // Interface for moving group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Planning scene to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // BASIC INFO
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // Name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // List of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // PLANNING
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  // Get joint names for group
  std::vector<std::string> manipulator_joint_names;
  manipulator_joint_names = move_group.getJoints();

  // Set start state to the current robot state
  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(1.0);

  /*

  //set target position for each joint
  std::map<std::string, double> target_position;

  target_position["shoulder_pan_joint"] = 0.0;
  target_position["shoulder_lift_joint"] = -1.5707;
  target_position["elbow_joint"] = 0.0;
  target_position["wrist_1_joint"] = -1.5707;
  target_position["wrist_2_joint"] = 0.0;
  target_position["wrist_3_joint"] = 0.0;

  for (unsigned int i = 0; i < manipulator_joint_names.size(); ++i)
    if (target_position.count(manipulator_joint_names[i]) > 0)
    {
      ROS_INFO_STREAM("\t" << manipulator_joint_names[i] << " goal position: " << target_position[manipulator_joint_names[i]]);
      move_group.setJointValueTarget(manipulator_joint_names[i], target_position[manipulator_joint_names[i]]);
    }

  /**/

  /**/

  move_group.setNamedTarget("rest_up_left");

  /**/

  /*

  //Pose computed with respect to world frame
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.70701;
  target_pose1.orientation.y = 0.70718;
  target_pose1.orientation.z = -0.0064114;
  target_pose1.orientation.w = -0.0064094;
  target_pose1.position.x = 0.81536;
  target_pose1.position.y = 0.19164;
  target_pose1.position.z = 0.85061;
  //move_group.setPoseReferenceFrame("world");
  //ROS_INFO("Pose Reference Frame: ", move_group.getPoseReferenceFrame());

  move_group.setPoseTarget(target_pose1, "ee_link");

  /**/

  // Try to plan movement from start to target position
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPlanningTime(5.0);
  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

  // If a plan is found execute it
  if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  move_group.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  ros::Duration(15.0).sleep();

  // Now, let's attach the collision object to the robot.
  ROS_INFO("Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  ros::Duration(15.0).sleep();

  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  ros::Duration(15.0).sleep();

  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  ros::Duration(15.0).sleep();

  spinner.stop();

  return 0;
}
