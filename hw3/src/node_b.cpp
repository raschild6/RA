#include "node_b.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose current_amcl_pose;
geometry_msgs::PoseStamped current_goal_map_pose;

bool amcl_set = false;
int goal_plan_status = -1;


void sendMyGoal(geometry_msgs::PoseStamped target_pose){
  MoveBaseClient ac("marrtino/move_base", true);
  move_base_msgs::MoveBaseGoal goal;
  
  /*
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped target_pose_tf;

  ros::Duration timeout(5);
  try
  {
    transformStamped = tfBuffer.lookupTransform("marrtino_map", target_pose.header.frame_id, ros::Time(0), timeout);
    tf2::doTransform(target_pose, target_pose_tf, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_INFO("Error Trasformation...%s", ex.what());
  }
  
  goal.target_pose = target_pose_tf;
  */
  goal.target_pose = target_pose;
  goal.target_pose.header.frame_id = "marrtino_map";
  goal.target_pose.header.stamp = ros::Time::now();

  ROS_INFO("Goal pose (marrtino_map): [%f, %f, %f] - [%f, %f, %f, %f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
      goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, 
      goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
  ROS_INFO(" ----- Sending goal -----");

  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("ARRIVED!");
  else
    ROS_INFO("FAILED!");

}

void resultFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &feedback_msgs){

  move_base_msgs::MoveBaseActionFeedback  feedback = *feedback_msgs;
  goal_plan_status = feedback.status.status;
}

void correctPoseWRTOdom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose_msgs){
  
  geometry_msgs::PoseWithCovarianceStamped amcl_pose_stamped = *amcl_pose_msgs;
  geometry_msgs::Pose amcl_pose = amcl_pose_stamped.pose.pose;
  
  if(!amcl_set){
    current_amcl_pose = amcl_pose;
    amcl_set = true;
    ROS_INFO("Current AMCL pose : [%f, %f, %f] - [%f, %f, %f, %f]", amcl_pose.position.x,
        amcl_pose.position.y, amcl_pose.position.z,
        amcl_pose.orientation.x, amcl_pose.orientation.y,
        amcl_pose.orientation.z, amcl_pose.orientation.w); 
    return;
  }

  ROS_INFO("New AMCL pose : [%f, %f, %f] - [%f, %f, %f, %f]", amcl_pose.position.x,
        amcl_pose.position.y, amcl_pose.position.z,
        amcl_pose.orientation.x, amcl_pose.orientation.y,
        amcl_pose.orientation.z, amcl_pose.orientation.w); 

  current_goal_map_pose.pose.position.x += amcl_pose.position.x - current_amcl_pose.position.x;
  current_goal_map_pose.pose.position.y += amcl_pose.position.y - current_amcl_pose.position.y;
  current_goal_map_pose.pose.position.z += amcl_pose.position.z - current_amcl_pose.position.z;
  
  tf2::Quaternion q_current_goal, q_old_odom, q_odom, q_translation;
  tf2::fromMsg(current_goal_map_pose.pose.orientation, q_current_goal);
  tf2::fromMsg(current_amcl_pose.orientation, q_old_odom);
  tf2::fromMsg(amcl_pose.orientation, q_odom);
  q_translation = q_odom * q_old_odom.inverse();
  
  ROS_INFO("Relative traslation values: [%f, %f, %f] - [%f, %f, %f, %f]", amcl_pose.position.x - current_amcl_pose.position.x, 
            amcl_pose.position.y - current_amcl_pose.position.y, amcl_pose.position.z - current_amcl_pose.position.z, 
            q_translation.getX(), q_translation.getY(), q_translation.getZ(), q_translation.getW());

  current_goal_map_pose.pose.orientation = tf2::toMsg(q_translation * q_current_goal);
  
  current_amcl_pose.position = amcl_pose.position;  
  current_amcl_pose.orientation = amcl_pose.orientation;

  current_goal_map_pose.header.frame_id = "marrtino_map";
  current_goal_map_pose.header.stamp = ros::Time::now();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Pose target_pose_tf;
  
  ros::Duration timeout(10);
  try{
    transformStamped = tfBuffer.lookupTransform("marrtino_base_footprint", "marrtino_map", ros::Time(0), timeout);
    tf2::doTransform(current_goal_map_pose.pose, target_pose_tf, transformStamped);
  }
  catch (tf2::TransformException &ex){
    ROS_INFO("Error Trasformation...%s", ex.what());
  }

  current_goal_map_pose.pose = target_pose_tf;
  current_goal_map_pose.header.frame_id = "marrtino_base_footprint";
  current_goal_map_pose.header.stamp = ros::Time::now();

  ROS_INFO("-> Goal pose update (marrtino_base_footprint): [%f, %f, %f] - [%f, %f, %f, %f]", current_goal_map_pose.pose.position.x, current_goal_map_pose.pose.position.y,
      current_goal_map_pose.pose.position.z, current_goal_map_pose.pose.orientation.x, current_goal_map_pose.pose.orientation.y, 
      current_goal_map_pose.pose.orientation.z, current_goal_map_pose.pose.orientation.w);

  sendMyGoal(current_goal_map_pose);
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  
  ros::Rate r(50);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  //ros::Subscriber odometry_marrtino = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, currentOdometry);
  ros::Subscriber feedback = n.subscribe("/marrtino/move_base/feedback", 1, resultFeedback);
  //ros::Subscriber correct_goal_pose = n.subscribe("/marrtino/amcl_pose", 1, correctPoseWRTOdom);

  current_goal_map_pose.header.frame_id = "marrtino_map";
  current_goal_map_pose.header.stamp = ros::Time::now();
  current_goal_map_pose.pose.position.x = -0.223;
  current_goal_map_pose.pose.position.y = 1.034;
  current_goal_map_pose.pose.position.z = 0.0;
  current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
  
  // --- GREEN final platform left
  //target_pose.position.x = 0.101878;
  //target_pose.position.y = 0.557094;
  //target_pose.position.z = 1.3;
  // --- GREEN final platform right
  //target_pose.position.x = -0.482639;
  //target_pose.position.y = 0.565774;
  //target_pose.position.z = 1.2;
  
  
  ROS_INFO("Fist Goal pose (marrtino_map): [%f, %f, %f] - [%f, %f, %f, %f]", current_goal_map_pose.pose.position.x, current_goal_map_pose.pose.position.y,
      current_goal_map_pose.pose.position.z, current_goal_map_pose.pose.orientation.x, current_goal_map_pose.pose.orientation.y, 
      current_goal_map_pose.pose.orientation.z, current_goal_map_pose.pose.orientation.w);

  sendMyGoal(current_goal_map_pose);
  
  r.sleep();

  /** Status code /
    uint8 status
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server
  /**/

  while(ros::ok()){
    if(goal_plan_status != -1 && goal_plan_status != 1){
      ROS_INFO("Goal Status: %d", goal_plan_status);
      r.sleep();
    }
    /*else if(goal_plan_status == 1){
      r.sleep();
    }else if(goal_plan_status == 3){
      current_goal_map_pose.pose.position.y += 0.5;
      if(current_goal_map_pose.pose.position.y < 2.7)
        sendMyGoal(current_goal_map_pose);
      else
        return 0;
    }else{
      //ROS_INFO("Goal Status: %d", goal_plan_status);
    }*/
  }

  ros::waitForShutdown();
  
  return 0;
}
