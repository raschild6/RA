#include "node_a.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_broadcast_marrtino");
  ros::NodeHandle n;
  ros::Rate rate(1000);
  ros::Publisher odom_map_tf = n.advertise<tf2_msgs::TFMessage>("marrtino/amcl/tf", 1);

  tf2_msgs::TFMessage tf_message;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  ros::Duration timeout(0.1);

  while (ros::ok())
  {
    try
    {
      if (tfBuffer.canTransform("marrtino_map", "marrtino_odom", ros::Time::now(), ros::Duration(3.0)))
      {
        transformStamped = tfBuffer.lookupTransform("marrtino_map", "marrtino_odom", ros::Time(0), timeout);
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_INFO("Error Trasformation...%s", ex.what());
    }

    tf_message.transforms = {transformStamped};
    //tf_message.transforms[0].header.frame_id = "marrtino_odom";
    //tf_message.transforms[0].child_frame_id = "marrtino_map";
    //tf::quaternionTFToMsg(tf::Quaternion(0.015, 0.000, 0.005, 1.000), tf_message.transforms[0].transform.rotation);
    //tf::vector3TFToMsg(tf::Vector3(-0.135, -0.044, -0.028), tf_message.transforms[0].transform.translation);
    odom_map_tf.publish(tf_message);
    ros::spinOnce();

  }
  return 0;
}
