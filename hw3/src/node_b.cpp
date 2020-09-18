#include "node_b.h"

using namespace std;

/**** GOAL ****/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose current_amcl_pose;
geometry_msgs::PoseStamped current_goal_map_pose;

bool amcl_set = false;
int goal_plan_status = -1;


/**** LASER ****/
ros::Publisher cmd_pub;
float range_min = 0;
float range_max = 0;
unordered_map<string, tuple<float, float>> regions;  // name_region, < min_value, mean_value >
int global_state = 0;
float min_obstacle_dist = 0.05, min_available_reg = 0.5;

/*
    Laser Scan: cycle [0:400]
                degree ->   back    =   [350:50]
                            right   =   [50:150]
                            front   =   [150:250]
                            left    =   [250:350]

        - robot stake:  back-right  =   [42:44]
                        front-right =   [111:115]
                        front-left  =   [286:290]
                        back-left   =   [357:359]
*/

void initMap()
{
  // each of 4 regions has a first part(1) and next part(2) of rays
  regions["right_1"] = tuple<float,float>(-1, -1);
  regions["right_2"] = tuple<float,float>(-1, -1);
  regions["front_1"] = tuple<float,float>(-1, -1);
  regions["front_2"] = tuple<float,float>(-1, -1);
  regions["left_1"] = tuple<float,float>(-1, -1);
  regions["left_2"] = tuple<float,float>(-1, -1);
  regions["back_1"] = tuple<float,float>(-1, -1);
  regions["back_2"] = tuple<float,float>(-1, -1);
}

void change_state(int state)
{
    ROS_INFO("CHANGE_STATE");
    if (state != global_state)
    {
        ROS_INFO("Wall follower - [%s]", state);
        global_state = state;
    }
}
void take_action(){

  /*
  ROS_INFO("TAKE ACTION");
  ROS_INFO("right : %f", regions["right"]);
  ROS_INFO("fright : %f", regions["fright"]);
  ROS_INFO("front : %f", regions["front"]);
  ROS_INFO("left : %f", regions["left"]);
  ROS_INFO("fleft : %f", regions["left"]);
  * /
  string state_description = "";

  float d = 0.3;

  if (regions["front"] > d && regions["left"] > d && regions["right"] > d)
  {
      state_description = "case 1 - nothing";
      change_state(0);
  }
  else if (regions["front"] < d && regions["left"] > d && regions["right"] > d)
  {
      state_description = "case 2 -front";
      change_state(1);
  }
  else if (regions["front"] > d && regions["left"] > d && regions["right"] < d)
  {
      state_description = "case 3- fright";
      change_state(2);
  }
  else if (regions["front"] > d && regions["left"] < d && regions["right"] > d)
  {
      state_description = "case 4 - fleft";
      change_state(0);
  }
  else if (regions["front"] < d && regions["left"] > d && regions["right"] < d)
  {
      state_description = "case 5 - front and fright";
      change_state(1);
  }
  else if (regions["front"] < d && regions["left"] < d && regions["right"] > d)
  {
      state_description = "case 6 - front and fleft";
      change_state(1);
  }
  else if (regions["front"] < d && regions["left"] < d && regions["right"] < d)
  {
      state_description = "case 7 - front and fleft and fright";
      change_state(1);
  }
  else if (regions["front"] > d && regions["left"] < d && regions["right"] < d)
  {
      state_description = "case 8 - fleft and fright";
      change_state(0);
  }
  else
  {
      state_description = "unknown case";
  }
  */


  if(get<0>(regions["front_1"]) < min_obstacle_dist || get<0>(regions["front_2"]) < min_obstacle_dist){

    // stop curren goal planning
    // TODO ??????????????????????


    vector<string> probably_regions = {};

    // iterate over the regions
    for_each(regions.begin(), regions.end() , [](pair<string, tuple<float, float>> element){

      // choose possible regions -> without an obstable and with good average of free space
      if(get<0>(element.second) >= min_obstacle_dist && get<1>(element.second) >= min_available_reg){
          probably_regions.push_back(element.first);
      }
    });


    // FRONT_2 BUSY -> FRONT_1 FREE, check LEFT_2 
    if(regions.find("front_1") != regions.end()){
      if(regions.find("left_2") != regions.end()){
        
        state_description = "case 1 - nothing";     // enough space for traveling 
        change_state(0);
      
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT_1 BUSY -> FRONT_2 FREE, check RIGHT_1 
    else if(regions.find("front_2") != regions.end()){
      if(regions.find("right_1") != regions.end()){
        
        state_description = "case 1 - nothing";     // enough space for traveling 
        change_state(0);
      
      }else{
        // decidere che fare?
      }
    }

    // FRONT_1, FRONT_2 BUSY -> check RIGHT_1, RIGHT_2 
    else if(regions.find("right_1") != regions.end()){
      if(regions.find("right_2") != regions.end()){
        
        state_description = "case 1 - nothing";     // enough space for traveling 
        change_state(0);
      
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT_1, FRONT_2 BUSY -> check also LEFT_1, LEFT_2
    if(regions.find("left_1") != regions.end()){
      if(regions.find("left_2") != regions.end()){
        
        state_description = "case 1 - nothing";     // enough space for traveling 
        change_state(0);
      
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT, RIGHT, LEFT BUSY -> check BACK_1, BACK_2
    if(/* ne right ne left liberi*/){
      if(regions.find("back_1") != regions.end()){
        if(regions.find("back_2") != regions.end()){
          
          state_description = "case 1 - nothing";     // enough space for traveling 
          change_state(0);
        
        }else{
          // decidere che fare?
        }
      }
    }

  }

  

}

geometry_msgs::Twist find_wall()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = -0.1;
    return msg;
}
geometry_msgs::Twist turn_left()
{
    geometry_msgs::Twist msg;
    msg.angular.z = 0.1;
    return msg;
}
geometry_msgs::Twist follow_the_wall()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    return msg;
}
void laserReadCallback(const sensor_msgs::LaserScan &msg)
{
    //ROS_INFO("LASER READ");
    range_min = msg.range_min;
    range_max = msg.range_max;
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    vector<float> right, front, left, back = {};
    //ROS_INFO("LASER RAYS: %d", msg.ranges.size());
    for (int i = 0; i < msg.ranges.size(); i++){
      
      //ROS_INFO("msg at %d: %f", i, msg.ranges.at(i));

      // Skip robot stake ray
      if((i >= 42 && i <= 44) || (i >= 111 && i <= 115) || (i >= 286 && i <= 290) || (i >= 357 && i <= 359))
        continue;

      if (msg.ranges.at(i) > range_min){
        if (i < 1 / 8 * msg.ranges.size() || i >= 7 / 8 * msg.ranges.size())
            back.push_back(msg.ranges.at(i));
        if (i < 3 / 8 * msg.ranges.size() || i >= 1 / 8 * msg.ranges.size())
            right.push_back(msg.ranges.at(i));
        if (i < 5 / 8 * msg.ranges.size() || i >= 3 / 8 * msg.ranges.size())
            front.push_back(msg.ranges.at(i));
        if (i < 7 / 8 * msg.ranges.size() || i >= 5 / 8 * msg.ranges.size())
            left.push_back(msg.ranges.at(i));
      }
    }
    
    //ROS_INFO("RIGHT SIZE: %d", right.size());
    //ROS_INFO("MIN: %f", *min_element(begin(right), end(right)));
    //ROS_INFO("FRONT SIZE: %d", front.size());
    //ROS_INFO("MIN: %f", *min_element(begin(right), end(right)));
    //ROS_INFO("LEFT SIZE: %d", left.size());
    //ROS_INFO("MIN: %f", *min_element(begin(right), end(right)));
    //ROS_INFO("BACK SIZE: %d", back.size());
    //ROS_INFO("MIN: %f", *min_element(begin(back), end(back)));
    
    regions["right_1"] = tuple<float,float>(*min_element(begin(right), begin(right) + (right.size() / 2) - 1), accumulate(begin(right), begin(right) + (right.size() / 2) - 1, 0.0) / right.size());
    regions["right_2"] = tuple<float,float>(*min_element(begin(right) + right.size() / 2, end(right)), accumulate(begin(right) + right.size() / 2, end(right), 0.0) / right.size());
    regions["front_1"] = tuple<float,float>(*min_element(begin(front), begin(front) + (front.size() / 2) - 1), accumulate(begin(front), begin(front) + (front.size() / 2) - 1, 0.0) / front.size());
    regions["front_2"] = tuple<float,float>(*min_element(begin(front) + front.size() / 2, end(front)), accumulate(begin(front) + front.size() / 2, end(front), 0.0) / front.size());
    regions["left_1"] = tuple<float,float>(*min_element(begin(left), begin(left) + (left.size() / 2) - 1), accumulate(begin(left), begin(left) + (left.size() / 2) - 1, 0.0) / left.size());
    regions["left_2"] = tuple<float,float>(*min_element(begin(left) + left.size() / 2, end(left)), accumulate(begin(left) + left.size() / 2, end(left), 0.0) / left.size());
    regions["back_2"] = tuple<float,float>(*min_element(begin(back), begin(back) + (back.size() / 2) - 1), accumulate(begin(back), begin(back) + (back.size() / 2) - 1, 0.0) / back.size());
    regions["back_1"] = tuple<float,float>(*min_element(begin(back) + back.size() / 2, end(back)), accumulate(begin(back) + back.size() / 2, end(back), 0.0) / back.size());
    // NB. back is inverted obv.
    
    take_action();
}


void sendMyGoal(geometry_msgs::PoseStamped target_pose){
  MoveBaseClient ac("marrtino/move_base", true);
  move_base_msgs::MoveBaseGoal goal;
  
   //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

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

  //current_goal_map_pose.pose.orientation = tf2::toMsg(q_translation * q_current_goal);
  
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
  ros::Subscriber sub_laser = n.subscribe("/marrtino/scan", 1, laserReadCallback);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1);
  
  
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
    }
    if(current_goal_map_pose.pose.position.y > 2.7){
      ROS_INFO(" ------------ END Narrow Passage ------------");
      //correct_goal_pose.shutdown();
      break;
    }

  initMap();
  ros::Duration(1).sleep();
  ros::Rate rate(100);
  while (ros::ok())
  {
      geometry_msgs::Twist msg;
      if (global_state == 0)
          msg = find_wall();
      else if (global_state == 1)
          msg = turn_left();
      else if (global_state == 2)
      {
          msg = follow_the_wall();
          continue;
      }
      else
          ROS_INFO("Unknown state!");

      cmd_pub.publish(msg);
      ros::spinOnce();

      rate.sleep();
  }




    //r.sleep();
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
