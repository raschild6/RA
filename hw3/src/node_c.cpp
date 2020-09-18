#include "node_a.h"

ros::Publisher pose_pub;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
  ROS_INFO("\t\t- Localization pose(x, y) = [%f, %f]", msgAMCL->pose.pose.position.x, msgAMCL->pose.pose.position.y);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped source_pose_tf;
  source_pose_tf.header = msgAMCL->header;
  source_pose_tf.pose = msgAMCL->pose.pose;
  ROS_INFO("\t\t- Source pose(x, y) = [%f, %f]", source_pose_tf.pose.position.x, source_pose_tf.pose.position.y);

  geometry_msgs::PoseStamped target_pose_tf;

  ros::Duration timeout(0.1);
  try
  {
    transformStamped = tfBuffer.lookupTransform("marrtino_odom", "marrtino_map", ros::Time(0), timeout);
    //ROS_INFO("Translation x: %f", transformStamped.transform.translation.x);
    tf2::doTransform(source_pose_tf, target_pose_tf, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_INFO("Error Trasformation...%s", ex.what());
  }
  ROS_INFO("\t\t- Transformed Localization pose(x, y) = [%f, %f]", target_pose_tf.pose.position.x, target_pose_tf.pose.position.y);
}

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr &msgOdom)
{
  ROS_INFO("\t\t- Odometry pose(x, y) = [%f, %f]", msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y);
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose = msgOdom->pose.pose;
  pose.header.frame_id = "marrtino_odom";
  pose_pub.publish(pose);
}

void gazeboPoseCallback(const gazebo_msgs::ModelStates &model_states)
{
  for (int i = 0; i < model_states.name.size(); i++)
  {
    if (model_states.name[i] == "marrtino::marrtino_base_footprint")
    {
      ROS_INFO("\t\t- Gazebo pose(x, y) = [%f, %f]", model_states.pose.at(i).position.x, model_states.pose.at(i).position.y);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "print_martino_pose");
  ros::AsyncSpinner async_spinner_glob(0);
  ros::NodeHandle n;
  ros::Subscriber sub_amcl = n.subscribe("/marrtino/amcl_pose", 1, amclPoseCallback);
  ros::Subscriber sub_marrtino = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, odomPoseCallback);
  ros::Subscriber sub_gazebo = n.subscribe("/gazebo/link_states", 1, gazeboPoseCallback);
  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/marrtino/initial_pose", 1);

  async_spinner_glob.start();

  ros::waitForShutdown();

  return 0;
}
