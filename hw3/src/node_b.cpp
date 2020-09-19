#include "node_b.h"

using namespace std;

/**** GOAL ****/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose current_amcl_pose;
geometry_msgs::PoseStamped current_goal_map_pose;

bool amcl_set = false;
int goal_plan_status = -10;


/**** LASER ****/
ros::Publisher cmd_pub;
float range_min = 0;
float range_max = 0;
unordered_map<string, tuple<float, float>> regions;  // name_region, < min_value, mean_value >
int global_state = 0;
float min_obstacle_dist = 0.35, min_available_reg = 0.7;
bool action_in_progress = false;
int action_step = 0;

/*
    Laser Scan: cycle [0:400]
                degree ->   back    =   [350:50]
                            right   =   [50:150]
                            front   =   [150:250]
                            left    =   [250:350]

        - robot stake:  back-right  =   [41:43]
                        front-right =   [110:114]
                        front-left  =   [285:289]
                        back-left   =   [356:358]
        (NB. start from 0 not 1) 
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

  if(get<0>(regions["front_1"]) < min_obstacle_dist || get<0>(regions["front_2"]) < min_obstacle_dist){

    vector<string> probably_regions = {};
    vector<int> space_states = {};
    int come_back = 0;

    // iterate over the regions
    for_each(regions.begin(), regions.end() , [&probably_regions](pair<string, tuple<float, float>> element){

      // choose possible regions -> without an obstable and with good average of free space
      //if(get<0>(element.second) >= min_obstacle_dist && get<1>(element.second) >= min_available_reg){
      if(get<0>(element.second) >= min_available_reg){
          probably_regions.push_back(element.first);
      }
    });


    // FRONT_2 BUSY -> FRONT_1 FREE, check RIGHT_2 
    if(find(probably_regions.begin(), probably_regions.end(),"front_1") != probably_regions.end() && find(probably_regions.begin(), probably_regions.end(),"front_2") == probably_regions.end()){
      if(find(probably_regions.begin(), probably_regions.end(),"right_2") != probably_regions.end()){
        
        ROS_INFO("-----> pass on the front-right");     // enough space for traveling 
        space_states.push_back(-1);
      
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT_1 BUSY -> FRONT_2 FREE, check LEFT_1 
    else if(find(probably_regions.begin(), probably_regions.end(),"front_2") != probably_regions.end() && find(probably_regions.begin(), probably_regions.end(),"front_1") == probably_regions.end()){
      if(find(probably_regions.begin(), probably_regions.end(),"left_1") != probably_regions.end()){
        
        ROS_INFO("-----> pass on the front-left");     // enough space for traveling 
        space_states.push_back(-2);
      
      }else{
        // decidere che fare?
      }
    }

    // FRONT_1, FRONT_2 BUSY -> check RIGHT_1, RIGHT_2 
    else if(find(probably_regions.begin(), probably_regions.end(),"right_1") != probably_regions.end()){
      if(find(probably_regions.begin(), probably_regions.end(),"right_2") != probably_regions.end()){
        
        ROS_INFO("-----> turn right");     // enough space for traveling 
        space_states.push_back(-3);
        come_back++;
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT_1, FRONT_2 BUSY -> check also LEFT_1, LEFT_2
    if(find(probably_regions.begin(), probably_regions.end(),"left_1") != probably_regions.end()){
      if(find(probably_regions.begin(), probably_regions.end(),"left_2") != probably_regions.end()){
        
        ROS_INFO("-----> turn_left");     // enough space for traveling 
        space_states.push_back(-4);
        come_back++;
      }else{
        // decidere che fare?
      }
    }
    
    // FRONT, RIGHT, LEFT BUSY -> check BACK_1, BACK_2
    if(come_back == 0){
      if(find(probably_regions.begin(), probably_regions.end(),"back_1") != probably_regions.end()){
        if(find(probably_regions.begin(), probably_regions.end(),"back_2") != probably_regions.end()){
          
          ROS_INFO("-----> go back");     // enough space for traveling 
          space_states.push_back(-5);
        
        }else{
          // decidere che fare?
        }
      }
    }

    ROS_INFO("STATES RECOGNICE:");
    for(int state : space_states){
      ROS_INFO("\t\t state: %d", state);
    }
    
    if(space_states.size() != 0)
      goal_plan_status = space_states[0];   // per ora scegli la prima soluzione trovata
      action_in_progress = true;
    else
      ROS_INFO("space_states empty");
  }

  

}

bool turn_right_plan(){
  geometry_msgs::Twist msg;
  
  msg = turn_right();
  ros::Time rotate_time_start = ros::Time::now();
  cmd_pub.publish(msg);
  
  // rotate robot until front object disappear from front_1/front_2 
  while(action_step == 1){
    ros::Duration(1).sleep();
  }
  ros::Time rotate_time = rotate_time_start - ros::Time::now();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Rotation right completed");

  msg = go_straight_ahead();
  cmd_pub.publish(msg);
  ros::Duration(1).sleep();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Go ahead completed");

  msg = turn_left();
  cmd_pub.publish(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Rotation left_back completed");

  return true;
}
bool turn_left_plan(){
  geometry_msgs::Twist msg;
  
  msg = turn_left();
  ros::Time rotate_time_start = ros::Time::now();
  cmd_pub.publish(msg);
  
  // rotate robot until front object disappear from front_1/front_2 
  while(action_step == 1){
    ros::Duration(1).sleep();
  }
  ros::Time rotate_time = rotate_time_start - ros::Time::now();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Rotation left completed");

  msg = go_straight_ahead();
  cmd_pub.publish(msg);
  ros::Duration(1).sleep();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Go ahead completed");

  msg = turn_right();
  cmd_pub.publish(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  cmd_pub.publish(msg);
  ROS_INFO("Rotation right_back completed");

  return true;
}

geometry_msgs::Twist turn_right()
{
  geometry_msgs::Twist msg;
  msg.angular.z = -0.2;
  return msg;
}
geometry_msgs::Twist turn_left()
{
  geometry_msgs::Twist msg;
  msg.angular.z = 0.2;
  return msg;
}
geometry_msgs::Twist go_straight_ahead()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.5;
  return msg;
}
geometry_msgs::Twist done()
{
  geometry_msgs::Twist msg;
  twist_msg.linear.x = 0;
  twist_msg.angular.z = 0;
  return msg;
}

void laserReadCallback(const sensor_msgs::LaserScan &msg){
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
    if(i == 0 || (i >= 41 && i <= 43) || (i >= 110 && i <= 114) || (i >= 285 && i <= 289) || (i >= 356 && i <= 358))
      continue;

    if (msg.ranges.at(i) > range_min){
      if(msg.ranges.at(i) > 12.0){
        if ((i >= 0 && i < msg.ranges.size() * 1/8) || (i >= msg.ranges.size() * 7/8 && i < msg.ranges.size()))
            back.push_back(12.0);
        else if (i < msg.ranges.size() * 3/8 && i >= msg.ranges.size() * 1/8)
            right.push_back(12.0);
        else if (i < msg.ranges.size() * 5/8 && i >= msg.ranges.size() * 3/8)
            front.push_back(12.0);
        else if (i < msg.ranges.size() * 7/8 && i >= msg.ranges.size() * 5/8)
            left.push_back(12.0);

      }else{

        if ((i >= 0 && i < msg.ranges.size() * 1/8) || (i >= msg.ranges.size() * 7/8 && i < msg.ranges.size()))
            back.push_back(msg.ranges.at(i));
        else if (i < msg.ranges.size() * 3/8 && i >= msg.ranges.size() * 1/8)
            right.push_back(msg.ranges.at(i));
        else if (i < msg.ranges.size() * 5/8 && i >= msg.ranges.size() * 3/8)
            front.push_back(msg.ranges.at(i));
        else if (i < msg.ranges.size() * 7/8 && i >= msg.ranges.size() * 5/8)
            left.push_back(msg.ranges.at(i));
      }
    }
  }
  

  
  regions["right_1"] = tuple<float,float>(*min_element(begin(right), begin(right) + (right.size() / 2) - 1), accumulate(begin(right), begin(right) + (right.size() / 2) - 1, 0.0) / (right.size() / 2));
  regions["right_2"] = tuple<float,float>(*min_element(begin(right) + right.size() / 2, end(right)), accumulate(begin(right) + right.size() / 2, end(right), 0.0) / (right.size() / 2));
  regions["front_1"] = tuple<float,float>(*min_element(begin(front), begin(front) + (front.size() / 2) - 1), accumulate(begin(front), begin(front) + (front.size() / 2) - 1, 0.0) / (front.size() / 2));
  regions["front_2"] = tuple<float,float>(*min_element(begin(front) + front.size() / 2, end(front)), accumulate(begin(front) + front.size() / 2, end(front), 0.0) / (front.size() / 2));
  regions["left_1"] = tuple<float,float>(*min_element(begin(left), begin(left) + (left.size() / 2) - 1), accumulate(begin(left), begin(left) + (left.size() / 2) - 1, 0.0) / (left.size() / 2));
  regions["left_2"] = tuple<float,float>(*min_element(begin(left) + left.size() / 2, end(left)), accumulate(begin(left) + left.size() / 2, end(left), 0.0) / (left.size() / 2));
  regions["back_2"] = tuple<float,float>(*min_element(begin(back), begin(back) + (back.size() / 2) - 1), accumulate(begin(back), begin(back) + (back.size() / 2) - 1, 0.0) / (back.size() / 2));
  regions["back_1"] = tuple<float,float>(*min_element(begin(back) + back.size() / 2, end(back)), accumulate(begin(back) + back.size() / 2, end(back), 0.0) / (back.size() / 2));
  // NB. back is inverted obv.
  
  /** /
  ROS_INFO("RIGHT_1 SIZE: %d - min, mean: %f, %f", right.size(), get<0>(regions["right_1"]), get<1>(regions["right_1"]));
  ROS_INFO("RIGHT_2 SIZE: %d - min, mean: %f, %f", right.size(), get<0>(regions["right_2"]), get<1>(regions["right_2"]));
  ROS_INFO("FRONT_1 SIZE: %d - min, mean: %f, %f", front.size(), get<0>(regions["front_1"]), get<1>(regions["front_1"]));
  ROS_INFO("FRONT_2 SIZE: %d - min, mean: %f, %f", front.size(), get<0>(regions["front_2"]), get<1>(regions["front_2"]));
  ROS_INFO("LEFT_1  SIZE: %d - min, mean: %f, %f", left.size(), get<0>(regions["left_1"]), get<1>(regions["left_1"]));
  ROS_INFO("LEFT_2  SIZE: %d - min, mean: %f, %f", left.size(), get<0>(regions["left_2"]), get<1>(regions["left_2"]));
  ROS_INFO("BACK_1  SIZE: %d - min, mean: %f, %f", back.size(), get<0>(regions["back_1"]), get<1>(regions["back_1"]));
  ROS_INFO("BACK_2  SIZE: %d - min, mean: %f, %f", back.size(), get<0>(regions["back_2"]), get<1>(regions["back_2"]));
  /**/

  if(!action_in_progress){  
    take_action();
  }else{
    switch (goal_plan_status){

      case -1:
      break;
      case -2:
      break;
      case -3:
        // check front_1/front_2 free
        if(get<0>(regions["front_1"]) >= min_available_reg && get<0>(regions["front_2"]) >= min_available_reg)
          action_step = 1;
      break;
      case -4:
        if(get<0>(regions["front_1"]) >= min_available_reg && get<0>(regions["front_2"]) >= min_available_reg)
          action_step = 1;
      break;
      case -5:
      break;
      default:
      break;
    }
  }
}


void change_goal_state(int state){
  
  if (state != goal_plan_status){
    ROS_INFO("CHANGE_GOAL_STATE: %d", state);
    goal_plan_status = state;
  }
}

move_base_msgs::MoveBaseGoal getGoal(geometry_msgs::PoseStamped target_pose){

  move_base_msgs::MoveBaseGoal goal;
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

  return goal;

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

  //sendMyGoal(current_goal_map_pose);
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  
  ros::Rate r(10);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  initMap();

  //ros::Subscriber odometry_marrtino = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, currentOdometry);
  ros::Subscriber feedback = n.subscribe("/marrtino/move_base/feedback", 1, resultFeedback);
  //ros::Subscriber correct_goal_pose = n.subscribe("/marrtino/amcl_pose", 1, correctPoseWRTOdom);
  ros::Subscriber sub_laser = n.subscribe("/marrtino/scan", 1, laserReadCallback);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1);  
  
  MoveBaseClient ac("marrtino/move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


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

  move_base_msgs::MoveBaseGoal final_goal = getGoal(current_goal_map_pose);
  //ac.sendGoal(final_goal);

  r.sleep();

  /********* Status code *********
    
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

    goal_plan_status      = -1  # Pass to the right
                          = -2  # Pass to the left
                          = -3  # Turn right
                          = -4  # Turn left
                          = -5  # Go back
                          = -10 # Not initialized yet
  
  /*******************************

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("ARRIVED!");
  else
    ROS_INFO("FAILED!");

  /**/
  
  while(ros::ok()){
        
    switch (goal_plan_status){

      case -10:
        // State not initialized yet... wait
      break;
      case -1:
        //ac.cancelGoal();
        ROS_INFO("Goal plan Cancelled!");
        action_in_progress = false;
      break;
      case -2:
        //ac.cancelGoal();
        ROS_INFO("Goal plan Cancelled!");
        action_in_progress = false;
      break;
      case -3:
        //ac.cancelGoal();
        //ROS_INFO("Goal plan Cancelled!");
        ROS_INFO("Start turn right!");
        turn_right_plan();
        action_in_progress = false;
      break;
      case -4:
        //ac.cancelGoal();
        //ROS_INFO("Goal plan Cancelled!");
        ROS_INFO("Start turn left!");
        action_in_progress = false;
      break;
      case -5:
        //ac.cancelGoal();
        ROS_INFO("Goal plan Cancelled!");
        action_in_progress = false;
      break;
      case 0:
      
      break;
      case 1:
      
      break;
      case 2:
      
      break;
      case 3:
        ROS_INFO("Goal plan Reached!");
        return 0;
      break;
      case 4:
      
      break;
      case 5:
      
      break;
      case 6:
      
      break;
      case 7:
      
      break;
      case 8:
      
      break;
      case 9:
      
      break;
      default:
        ROS_INFO("Unknown state - goal_plan_status: %d!", goal_plan_status);
      break;
    }

    r.sleep();
  }

  ros::waitForShutdown();
  
  return 0;
}
