#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher initial_pose_pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(0); // Allow to get messages in async way (thread). 0 means: use all processor of machine
  spinner.start();

  initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("marrtino/move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward

  target_pose.pose.position.x = 1.0;
  target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped target_pose_tf;

  ros::Duration timeout(1);
  try
  {
    transformStamped = tfBuffer.lookupTransform("marrtino_map", "marrtino_base_footprint", ros::Time(0), timeout);
    tf2::doTransform(target_pose, target_pose_tf, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_INFO("Error Trasformation...%s", ex.what());
  }

  goal.target_pose.header.frame_id = "marrtino_map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose = target_pose_tf;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose.pose.pose.position.x = -1.31;
  initial_pose.pose.pose.position.y = 0;
  initial_pose.pose.pose.position.z = 0;
  tf2Scalar roll = 0;
  tf2Scalar pitch = 0;
  tf2Scalar yaw = 1.57;

  tf2::Quaternion rpy;
  rpy.setEuler(yaw, pitch, roll);
  tf2::convert(initial_pose.pose.pose.orientation, rpy);
  ;

  initial_pose_pub.publish(initial_pose);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  ros::waitForShutdown();
  return 0;
}
