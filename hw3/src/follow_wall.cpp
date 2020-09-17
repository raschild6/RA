#include "follow_wall.h"

ros::Publisher cmd_pub;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
double yaw;
double yaw_precision = M_PI / 90; // +/- 2 degree allowed
double dist_precision = 0.1;
int global_state = 0;

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr &msgOdom)
{
  //ROS_INFO("\t\t- Odometry pose(x, y) = [%f, %f]", msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y);
  robot_pose.pose.pose = msgOdom->pose.pose;
  robot_pose.header.frame_id = "marrtino_odom";
  tf::Pose current_goal;
  tf::poseMsgToTF(msgOdom->pose.pose, current_goal);
  yaw = tf::getYaw(current_goal.getRotation());
  return;
}

void change_state(int state)
{
  global_state = state;
  ROS_INFO("State changed to [%d]", state);
}

void fix_yaw(geometry_msgs::PoseStamped des_pose)
{
  double desired_yaw = atan2(des_pose.pose.position.y - robot_pose.pose.pose.position.y, des_pose.pose.position.x - robot_pose.pose.pose.position.x);
  double err_yaw = desired_yaw - yaw;

  geometry_msgs::Twist twist_msg;
  if (abs(err_yaw) > yaw_precision)
  {
    if (err_yaw > 0)
      twist_msg.angular.z = 0.7;
    else
      twist_msg.angular.z = -0.7;
    cmd_pub.publish(twist_msg);
  }

  if (abs(err_yaw) <= yaw_precision)
  {
    ROS_INFO("Yaw error: [%f]", err_yaw);
    change_state(1);
  }
}

void go_straight_ahead(geometry_msgs::PoseStamped des_pose)
{
  double desired_yaw = atan2(des_pose.pose.position.y - robot_pose.pose.pose.position.y, des_pose.pose.position.x - robot_pose.pose.pose.position.x);
  double err_yaw = desired_yaw - yaw;
  double err_pos = sqrt(pow(des_pose.pose.position.y - robot_pose.pose.pose.position.y, 2) + pow(des_pose.pose.position.x - robot_pose.pose.pose.position.x, 2));

  if (err_pos > dist_precision)
  {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.6;
    cmd_pub.publish(twist_msg);
  }
  else
  {
    ROS_INFO("Position error: [%f]", err_pos);
    change_state(2);
  }

  if (abs(err_yaw) > yaw_precision)
  {
    ROS_INFO("Yaw error: [%f]", err_yaw);
    change_state(0);
  }
}

void done()
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.angular.z = 0;
  cmd_pub.publish(twist_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_wall");
  ros::NodeHandle n;
  ros::Subscriber sub_odom = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, odomPoseCallback);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1);

  geometry_msgs::PoseStamped des_pose;
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward

  target_pose.pose.position.x = 1.0;
  target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  ros::Duration timeout(1);
  try
  {
    transformStamped = tfBuffer.lookupTransform("marrtino_map", "marrtino_base_footprint", ros::Time(0), timeout);
    tf2::doTransform(target_pose, des_pose, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_INFO("Error Trasformation...%s", ex.what());
  }

  des_pose.header.frame_id = "marrtino_map";
  des_pose.header.stamp = ros::Time::now();

  ros::Rate rate(100);
  while (ros::ok())
  {
    if (global_state == 0)
      fix_yaw(des_pose);
    else if (global_state == 1)
      go_straight_ahead(des_pose);
    else if (global_state == 2)
    {
      done();
      continue;
    }
    else
    {
      ROS_INFO("Unknown state!");
      continue;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
