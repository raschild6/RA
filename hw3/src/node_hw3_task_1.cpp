#include "node_hw3_task_1.h"

using namespace std;


// TODO: per semplificare l'uscita dalle piattaforme sarebbe da fargli fare una rotazione a M_PI/2 ma dai miei test con un pò di fatica ce la fa lo stesso


bool mode = false;    // false : narrow passages mode
                      // true  : open space mode

/**** GOAL open space variable ****/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose current_amcl_pose;
geometry_msgs::PoseStamped current_goal_map_pose;

bool amcl_set = false;
bool local_plan = false;
int goal_plan_status = -10, avoid_obj_plan_status = -10;
int id_goals = 0;
float find_obstacle = 0.75;
int free_right_platform = 0;

ros::Publisher wait_ur5_pub;
bool obj_loaded = false;    // wait until ur_5 load object to marrtino basket
bool hmw_4 = false;         // used to interact with ur_5 (pass '1' as argument) 

/**** LASER open space variables ****/
ros::Publisher cmd_pub;
ros::Publisher initpose_pub;
float range_min = 0;
float range_max = 0;
unordered_map<string, tuple<float, float>> regions;  // name_region, < min_value, average_value >
float min_obstacle_dist = 0.25, min_available_reg = 0.5, min_space_avail = 0.4, 
        lethal_dist_front = 0.16, lethal_dist_back = 0.18; // There is the body of marrtino  
bool action_in_progress = false;
int action_step = 0;
bool action_internal_condition = true;  // used to increase action_step for case -1, -2 (can't use if-else, it's an error)
int near_collision_local_ray [] = {-1, -1};     // use during local_plan (without costmaps) to avoid collision during motors actions
                                                      // {front_check_collision, back_check_collision}


/**** LASER narrow passages variables ****/
int global_narrow_state = -1;
int move_to_wall = 1;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
geometry_msgs::PoseStamped des_pose;
double yaw = 0;
double yaw_precision = M_PI / (90 * 2); // +/- 1 degree allowed
double dist_precision = 0.1;

bool stop_laser = false;
int narrow_action_step = 0;

int step_rotate_right = 0;
int step_rotate_left = 0;

int choose_gate = 0;  // 0 = not used, -2 = wait new clear laser scan, -1 = used but gate not chosen yet, 1-2 = gate1-gate2
int gate_saved = 0;

float d_front = 0.3;
float d_back = 0.1;

bool isComeBack = false;
int finish_narrow_mode_check = 0;

/*
  Laser Scan: cycle [1:400]
              degree ->   back    =   [350:50]
                          right   =   [50:150]
                          front   =   [150:250]
                          left    =   [250:350]

      - robot stake:  back-right  =   [41:43]     belongs back_2
                      front-right =   [110:114]   belongs right_2
                      front-left  =   [285:289]   belongs left_1
                      back-left   =   [356:358]   belongs back_1
      (NB. start from 0 not 1) 
*/


/**** LASER narrow passages methods ****/
void change_state(int state)
{
    //ROS_INFO("CHANGE_STATE %d", state);
    if (state != global_narrow_state)
    {
        //ROS_INFO("Wall follower - [%s]", state);
        global_narrow_state = state;
    }
}
void change_destination(){

  ros::Duration(0.3).sleep();
  switch (narrow_action_step){

    case -1:
    {
      if(gate_saved == 2){
        gate_saved = -1;
        stop_laser = true;
        des_pose.pose.position = robot_pose.pose.pose.position;
        des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI*8/13);

        des_pose.header.frame_id = "marrtino_map";
        des_pose.header.stamp = ros::Time(0); /**/
        ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
        ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
        tf::Pose current_goal;
        tf::poseMsgToTF(des_pose.pose, current_goal);

        ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
        change_state(1);
      }else{
        change_state(5);      
        finish_narrow_mode_check = 5;
        ROS_INFO("STATE 5 ACTIVATED - Finish narrow mode");
      }
    }
    break;
    case 0:
    {
      des_pose.pose.position.x = -1.327743;
      des_pose.pose.position.y = 2.9;  // 2.9 da me poi tocca nella rotazione 
      des_pose.pose.position.z = 0;
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(0);
    }
    break;
    case 1:
    {
      des_pose.pose.position = robot_pose.pose.pose.position;
      des_pose.pose.position.x = -0.64;
      des_pose.pose.orientation = robot_pose.pose.pose.orientation;

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(3);
    }
    break;
    case 2:
    {
      des_pose.pose.position = robot_pose.pose.pose.position;
      des_pose.pose.position.x = 1.17;
      des_pose.pose.orientation = robot_pose.pose.pose.orientation;

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(3);
    }
    break;
    case 3:
    {
      des_pose.pose.position.x = 1.23;
      des_pose.pose.position.y = 3.60;
      des_pose.pose.position.z = 0;
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(4);
    }
    break;
    case 4:
    {
      des_pose.pose.position.x = -1.2;
      des_pose.pose.position.y = 3.75;
      des_pose.pose.position.z = 0;
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(4);
    } 
    break;
    case 5:
    {
      stop_laser = true;
      if(step_rotate_right == 0){
          ROS_INFO("START ROTATE RIGHT IN STEPS");
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/3);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(1);
          
      }else if(step_rotate_right == 1){
          des_pose.pose.position.x = -1.24;
          des_pose.pose.position.y = 3.185;//3.16; 
          des_pose.pose.position.z = 0;
          des_pose.pose.orientation = robot_pose.pose.pose.orientation;

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(3);
      }else if(step_rotate_right == 2){
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(1);
      }
    }
    break;
    case 6:
    {
      stop_laser = true;
      if(step_rotate_left == 0){
          ROS_INFO("START ROTATE LEFT IN STEPS");
          des_pose.pose.position = robot_pose.pose.pose.position;
          //des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.004280);
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(4*M_PI/7);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
          
      }else if(step_rotate_left == 1){
          des_pose.pose.position.x = robot_pose.pose.pose.position.x - 0.13;
          des_pose.pose.position.y = robot_pose.pose.pose.position.y + 0.13; 
          des_pose.pose.position.z = 0;
          des_pose.pose.orientation = robot_pose.pose.pose.orientation;

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(3);
      }else if(step_rotate_left == 2){
          ROS_INFO("FINISH ROTATE LEFT IN PLACE");
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
      }
    }
    break;
    case 7:
    {
      stop_laser = true;
      if(step_rotate_right == 0){
          ROS_INFO("START ROTATE RIGHT IN STEPS");
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/7);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(1);
          
      }else if(step_rotate_right == 1){
          des_pose.pose.position.x = -0.6;
          des_pose.pose.position.y = 3.11; 
          des_pose.pose.position.z = 0;
          des_pose.pose.orientation = robot_pose.pose.pose.orientation;

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(3);
      }else if(step_rotate_right == 2){
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2 - 0.17);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(1);
      }
    }
    break;
    case 8:
    {
      stop_laser = true;
      des_pose.pose.position = robot_pose.pose.pose.position;
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(1);
    }
    break;
    case 9:
    {
      des_pose.pose.position = robot_pose.pose.pose.position;

      if(gate_saved == 1)
        des_pose.pose.position.y = 2.65;
      else if(gate_saved == 2)
        des_pose.pose.position.y = 2.41;
      else{
        ROS_INFO("WARNING: gate_saved not initialized");
        des_pose.pose.position.y = 2.65;
      }

      des_pose.pose.orientation = robot_pose.pose.pose.orientation;

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(3);
    }
    break;
    case 10:
    {      
      if(step_rotate_left == 0){
          ROS_INFO("START ROTATE LEFT IN PLACE");
          des_pose.pose.position = robot_pose.pose.pose.position;
          //des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.004280);
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-9*M_PI/10);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
          
      }else if(step_rotate_left == 1){
          des_pose.pose.position.x = robot_pose.pose.pose.position.x - 0.13;
          des_pose.pose.position.y = robot_pose.pose.pose.position.y - 0.13; 
          des_pose.pose.position.z = 0;
          des_pose.pose.orientation = robot_pose.pose.pose.orientation;

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(3);
      }else if(step_rotate_left == 2){
          ROS_INFO("FINISH ROTATE LEFT IN PLACE");
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
      }
      stop_laser = true;
    }    
    break;
    case 11:
    {      
      des_pose.pose.position.x = -1.327743;
      des_pose.pose.position.y = 0.18;
      des_pose.pose.position.z = 0;
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-3.130344);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(4);
    }
    break;
    case 12:
    { 
      /** /
      ROS_INFO("START ROTATE LEFT IN PLACE");
      des_pose.pose.position = robot_pose.pose.pose.position;
      //des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.004280);
      //des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-0.867799);
      des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

      des_pose.header.frame_id = "marrtino_map";
      des_pose.header.stamp = ros::Time(0); /**/
      ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
      ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);

      ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
      change_state(2);
      /**/
      if(step_rotate_left == 0){
          ROS_INFO("START ROTATE LEFT IN PLACE");
          des_pose.pose.position = robot_pose.pose.pose.position;
          //des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.004280);
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.087232); //-1.087232  -0.867799

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
          
      }else if(step_rotate_left == 1){
          des_pose.pose.position.x = robot_pose.pose.pose.position.x + 0.075;
          des_pose.pose.position.y = robot_pose.pose.pose.position.y - 0.075; 
          des_pose.pose.position.z = 0;
          des_pose.pose.orientation = robot_pose.pose.pose.orientation;

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(3);
      }else if(step_rotate_left == 2){
          ROS_INFO("FINISH ROTATE LEFT IN PLACE");
          des_pose.pose.position = robot_pose.pose.pose.position;
          des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.574898);

          des_pose.header.frame_id = "marrtino_map";
          des_pose.header.stamp = ros::Time(0); /**/
          ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
          ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
          tf::Pose current_goal;
          tf::poseMsgToTF(des_pose.pose, current_goal);

          ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));
          change_state(2);
      }
      /**/
      stop_laser = true;
    }
    break;

    default:
    break;
  }
}
void next_destination(int narrow_action)
{
  narrow_action_step = narrow_action;
  ROS_INFO("NEXT DESTINATION - state: %d", narrow_action_step);
  change_destination();
}

/**** COMMON methods ****/
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

void sendMotorCommand(geometry_msgs::Twist msg){
  cmd_pub.publish(msg);
  ros::spinOnce();
  if(mode)
    ros::Duration(0.1).sleep();

}

geometry_msgs::Twist find_wall_left()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = 0.1;
    return msg;
}
geometry_msgs::Twist find_wall_right()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = -0.1;
    return msg;
}
geometry_msgs::Twist turn_right()
{
  geometry_msgs::Twist msg;
  if(mode)
    msg.angular.z = -0.4;
  else
    msg.angular.z = -0.2;
  
  return msg;
}
geometry_msgs::Twist turn_left()
{
  geometry_msgs::Twist msg;
  if(mode)
    msg.angular.z = 0.4;
  else
    msg.angular.z = 0.2;
  return msg;
}
geometry_msgs::Twist turn_front_right()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.4;
  msg.angular.z = -0.4;
  return msg;
}
geometry_msgs::Twist turn_front_left()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.4;
  msg.angular.z = 0.4;
  return msg;
}
geometry_msgs::Twist go_straight()
{
  geometry_msgs::Twist msg;
  if(mode)
    msg.linear.x = 0.4;   
  else
    msg.linear.x = 0.2;
  return msg;
}
geometry_msgs::Twist go_back()
{
  geometry_msgs::Twist msg;
  msg.linear.x = -0.4;
  return msg;
}
geometry_msgs::Twist done()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  
  if(!mode){
    global_narrow_state = -1;
    ROS_INFO("GOAL REACHED");
    sendMotorCommand(msg);
    stop_laser = false;

    switch (narrow_action_step){

      case -1:
        // used to last rotation before exit from gate 2
      break;
        //after go to end corridor, rotate right in step
      case 0:
        narrow_action_step = 5;
      break;
        //after go to gate 1, if gate 1 free -> rotate right, else go to gate 2
      case 1:
      {
        if(choose_gate == 0)
          choose_gate = -2;
          
        if(choose_gate == 1){
          ROS_INFO("Start to go to gate 1");
          narrow_action_step = 7;
          gate_saved = 1;
          choose_gate = 0;
        }else if(choose_gate == 2){
          ROS_INFO("Start to go to gate 2");
          narrow_action_step = 2;
          gate_saved = 2;
          choose_gate = 0;
        }
      }
      break;
        //after go to gate 2, rotate right
      case 2:
          narrow_action_step = 8;
      break;
        //after go to left corner, rotate left in step
      case 3:
        narrow_action_step = 6;
        move_to_wall = 1;
        d_front = 0.27;
      break;
        //after go to right corner, rotate left in step
      case 4:
        narrow_action_step = 10;
        move_to_wall = 1;
        d_front = 0.3;
      break;
        //after rotate right, go to gate 1
      case 5:
        narrow_action_step = 1;
      break;
        //after rotate left, go to right corner
      case 6:
        narrow_action_step = 4;
      break;
        //after rotate right, go to end of gate1 / gate2
      case 7:
      case 8:
        narrow_action_step = 9;
      break;
        //after arrive to end gate1 / gate2, start open space mode
      case 9:
        narrow_action_step = -1;
      break;
        //after rotate left, go to home base
      case 10:
        narrow_action_step = 11;
        move_to_wall = 1;
      break;
        //after arriving at base, stop
      case 11:
        narrow_action_step = 12;
        break;
        //after arriving at home, rotate tu restart
      case 12:
        narrow_action_step = 0;
        move_to_wall = 1;
        break;
      
      default:
        ROS_INFO("ERROR!! narrow_action_step unknown = %d... start open space mode", narrow_action_step);
        narrow_action_step = -1;
      break;
    }
    next_destination(narrow_action_step);
  }
  
  return msg;
}

void initPoseAmcl(geometry_msgs::PoseStamped initpose){
  geometry_msgs::PoseWithCovarianceStamped initial_pose_cov;
  initial_pose_cov.header.frame_id = "marrtino_map";
  initial_pose_cov.header.stamp = ros::Time(0); /**/

  initial_pose_cov.pose.pose = initpose.pose;

  initpose_pub.publish(initial_pose_cov);
  ros::Duration(0.3).sleep();
}


/**** LASER narrow passages callback ****/
void take_narrow_action(){

  //float d = 0.17;  //good for min value

  // if choose_gate = -2 -> wait a clean laser scan
  if(choose_gate == -2){
    choose_gate = -1;
    return;
  }else 
  // if choose_gate = -1 -> looking for a free gate
  if(choose_gate == -1){
    double average_front = (get<1>(regions["front_1"]) + get<1>(regions["front_2"]));
    double average_right = (get<1>(regions["right_1"]) + get<1>(regions["right_2"]));
    double min_front = (get<0>(regions["front_1"]) + get<0>(regions["front_2"]));
    double min_right = (get<0>(regions["right_1"]) + get<0>(regions["right_2"]));
    
    ROS_INFO("CHOOSE GATE: average front = %f - average right = %f", average_front, average_right);
    ROS_INFO("CHOOSE GATE: min front = %f - min right = %f", min_front, min_right);
    if(average_front > 0.0 && average_right > 0.0){
      if (min_front < min_right){
        ROS_INFO("\t---> Gate1 chosen!");
        choose_gate = 1;   
      }else{
        ROS_INFO("\t---> Gate2 chosen!");
        choose_gate = 2;
      }
    }else{
      ROS_INFO("BOTH AVERAGES ZERO! there's some error?");
    }
    return;
  }

  if (get<1>(regions["left_1"]) < d_front && get<1>(regions["right_2"]) > d_front)
  {
    ROS_INFO("TAKE ACTION \t\t TURN RIGHT");

    move_to_wall = 0;
    change_state(1); //turn right

  }
  else if (get<1>(regions["left_1"]) > d_front && get<1>(regions["right_2"]) < d_front)
  {
    ROS_INFO("TAKE ACTION \t\t TURN LEFT");

    move_to_wall = 0;
    change_state(2); //turn left
  }
  else if (get<1>(regions["left_1"]) > d_front && get<1>(regions["right_2"]) > d_front)
  {
    if (move_to_wall)
    {
      if (narrow_action_step == 0){
        ROS_INFO("TAKE ACTION \t\t TURN LEFT TO WALL");
        change_state(0); //find wall by rotating left and going forward
      }
      else if (narrow_action_step == 3 || narrow_action_step == 4 || narrow_action_step == 11){
        ROS_INFO("TAKE ACTION \t\t TURN RIGHT TO WALL");
        change_state(4); //find wall by rotating left and going forward
      }
    }
    else
    {
      //ROS_INFO("TAKE ACTION \t\t GO STRAIGHT");
      change_state(3); //go straight forward
    }
  }

  
}
void check_goal(){

  if ((narrow_action_step >= 0 && narrow_action_step <= 4 && narrow_action_step != 2) || narrow_action_step == 9 || narrow_action_step == 11){
      double desired_yaw = atan2(des_pose.pose.position.y - robot_pose.pose.pose.position.y, des_pose.pose.position.x - robot_pose.pose.pose.position.x);
      double err_yaw = desired_yaw - yaw;
      double err_pos = sqrt(pow(des_pose.pose.position.y - robot_pose.pose.pose.position.y, 2) + pow(des_pose.pose.position.x - robot_pose.pose.pose.position.x, 2));
      //ROS_INFO("DESIRED YAW: %f", desired_yaw);

      //if (err_pos < dist_precision && err_yaw < yaw_precision)
      if (err_pos < dist_precision)
        done();
  }else if(narrow_action_step == 2){
      if (des_pose.pose.position.x - robot_pose.pose.pose.position.x < dist_precision)
        done();
  }
  else if (narrow_action_step == 5 || narrow_action_step == 7){
      tf::Pose current_goal;
      tf::poseMsgToTF(des_pose.pose, current_goal);
      tf::Pose current_pose;
      tf::poseMsgToTF(robot_pose.pose.pose, current_pose);
      
      // done twist
      geometry_msgs::Twist msg;
      msg.linear.x = 0;
      msg.angular.z = 0;    
      
      if(step_rotate_right == 0 || step_rotate_right == 2){
          if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.1){
              
              if(step_rotate_right == 0) 
                  ROS_INFO("FIRST STEP TURN RIGHT COMPLETED"); 
              else 
                  ROS_INFO("THIRD STEP TURN RIGHT COMPLETED");
              
              step_rotate_right++;
              sendMotorCommand(msg);
              change_destination();
          }
      }else if(step_rotate_right == 1){
          double err_pos = sqrt(pow(des_pose.pose.position.y - robot_pose.pose.pose.position.y, 2) + pow(des_pose.pose.position.x - robot_pose.pose.pose.position.x, 2));
          if (err_pos < dist_precision){ 
              step_rotate_right++;

              ROS_INFO("SECOND GO AHEAD COMPLETED");
              sendMotorCommand(msg);
              change_destination();
          }
      }else{
          if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.1){
              done();
              step_rotate_right = 0;
          }
      }
  }
  else if (narrow_action_step == 6  || narrow_action_step == 10 || narrow_action_step == 12){
    tf::Pose current_goal;
    tf::poseMsgToTF(des_pose.pose, current_goal);
    tf::Pose current_pose;
    tf::poseMsgToTF(robot_pose.pose.pose, current_pose);
  
    // done twist
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
        
    if(step_rotate_left == 0 || step_rotate_left == 2){
      if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.01){
            
        if(step_rotate_left == 0) 
            ROS_INFO("FIRST STEP TURN RIGHT COMPLETED"); 
        else 
            ROS_INFO("THIRD STEP TURN RIGHT COMPLETED");
        
        step_rotate_left++;
        sendMotorCommand(msg);
        change_destination();
      }
    }else if(step_rotate_left == 1){
      double err_pos = sqrt(pow(des_pose.pose.position.y - robot_pose.pose.pose.position.y, 2) + pow(des_pose.pose.position.x - robot_pose.pose.pose.position.x, 2));
      if (err_pos < dist_precision){ 
        step_rotate_left++;

        ROS_INFO("SECOND GO AHEAD COMPLETED");
        sendMotorCommand(msg);
        change_destination();
      }
    }else{
      if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.1){
        done();
        step_rotate_left = 0;
      }
    }
  }
  else if (narrow_action_step == 8){
    tf::Pose current_goal;
    tf::poseMsgToTF(des_pose.pose, current_goal);
    tf::Pose current_pose;
    tf::poseMsgToTF(robot_pose.pose.pose, current_pose);

    if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.1)
      done();
  }
  else if (narrow_action_step == -1 && gate_saved == -1){
    tf::Pose current_goal;
    tf::poseMsgToTF(des_pose.pose, current_goal);
    tf::Pose current_pose;
    tf::poseMsgToTF(robot_pose.pose.pose, current_pose);

    if (fabs(tf::getYaw(current_pose.getRotation()) - tf::getYaw(current_goal.getRotation())) < 0.1){
      done();
      gate_saved = 0;
    }
  }
}
void odomPoseCallback(const nav_msgs::Odometry::ConstPtr &msgOdom)
{
    //ROS_INFO("\t\t- Odometry pose(x, y) = [%f, %f]", msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y);
    robot_pose.pose.pose = msgOdom->pose.pose;
    robot_pose.header.frame_id = "marrtino_map";
    tf::Pose current_goal;
    tf::poseMsgToTF(msgOdom->pose.pose, current_goal);
    yaw = tf::getYaw(current_goal.getRotation());
    //ROS_INFO("CURRENT YAW: %f", yaw);
    if(mode){
      geometry_msgs::PoseStamped endGate1Pose;
      endGate1Pose.header = robot_pose.header; 
      endGate1Pose.pose = robot_pose.pose.pose;
      initPoseAmcl(endGate1Pose);
    }else{
      check_goal();
    }
}


/**** LASER open space methods ****/
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
    if(find(probably_regions.begin(), probably_regions.end(),"right_1") != probably_regions.end()){
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
    
    if(space_states.size() != 0){

      action_in_progress = true;     
       
      // prefer to pass in front of right/left, if possible
      if(space_states[0] == -1 || space_states[0] == -2){
        avoid_obj_plan_status = space_states[0];
        ROS_INFO("Front-Right/Left free -> take it");
      }else{ 
        // if not, but right and left both free -> get one with highest average 
        if(come_back == 2){
          if(get<1>(regions["right_1"]) + get<1>(regions["right_2"]) > get<1>(regions["left_1"]) + get<1>(regions["left_2"])){
            avoid_obj_plan_status = -3;
            ROS_INFO("Right and Left free -> take Right, which has highest average");
          }else{
            avoid_obj_plan_status = -4;
            ROS_INFO("Right and Left free -> take Left, which has highest average");
          }
        }else{
          // only right or left is free -> take it
          // if only back is available -> take it anyway 
          avoid_obj_plan_status = space_states[0];
        }
      }

    }else{
      ROS_INFO("space_states empty");
    }
  }

}

bool turn_right_plan(){
  geometry_msgs::Twist msg;
  
  msg = go_back();
  sendMotorCommand(msg);
  ros::Duration(0.3).sleep();
  msg = done();
  sendMotorCommand(msg);
  ros::Duration(1).sleep();
  ROS_INFO("Back a little completed");
  
  msg = turn_right();
  double rotate_time_start = ros::Time::now().toSec();
  sendMotorCommand(msg);
  
  // rotate robot until front object disappear from front_1/front_2 
  while(action_step == 0){
    if(saveFatalCollision())
      return false;
    ros::Duration(0.3).sleep();
  }
  action_step = 0;
  double rotate_time = ros::Time::now().toSec() - rotate_time_start;
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Rotation right completed");

  msg = go_straight();
  sendMotorCommand(msg);
  ros::Duration(1).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Go ahead completed");

  msg = turn_left();
  sendMotorCommand(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Rotation left_back completed");

  ROS_INFO("Turn Right completed");
  
  return true;
}
bool turn_left_plan(){
  geometry_msgs::Twist msg;
  
  msg = go_back();
  sendMotorCommand(msg);
  ros::Duration(0.3).sleep();
  msg = done();
  sendMotorCommand(msg);
  ros::Duration(1).sleep();
  ROS_INFO("Back a little completed");
  
  msg = turn_left();
  double rotate_time_start = ros::Time::now().toSec();
  sendMotorCommand(msg);

  // rotate robot until front object disappear from front_1/front_2 
  while(action_step == 0){
    if(saveFatalCollision())
      return false;
    ros::Duration(0.3).sleep();
  }
  action_step = 0;
  double rotate_time = ros::Time::now().toSec() - rotate_time_start;
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Rotation left completed");

  msg = go_straight();
  sendMotorCommand(msg);
  ros::Duration(1).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Go ahead completed");

  msg = turn_right();
  sendMotorCommand(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Rotation right_back completed");

  ROS_INFO("Turn Left completed");

  return true;
}
bool turn_front_right_plan(){
  geometry_msgs::Twist msg;
  
  msg = go_back();
  sendMotorCommand(msg);
  ros::Duration(0.25).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Back a little completed");
  ros::Duration(0.5).sleep();

  msg = turn_right();
  double rotate_time_start = ros::Time::now().toSec();
  sendMotorCommand(msg);
  
  // wait until front object disappear from front_2 or timeout
  while(action_step != 1 && ros::Time::now().toSec() - rotate_time_start < 3.0){
    if(saveFatalCollision())
      return false;
    action_internal_condition = false;    // switch next action_step
    ros::Duration(0.3).sleep();
  }
  double rotate_time = ros::Time::now().toSec() - rotate_time_start;
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("End first right rotation completed");
  ros::Duration(0.5).sleep();

  double go_straight_time_start = ros::Time::now().toSec();
  msg = go_straight();
  sendMotorCommand(msg);
  ros::Duration(0.5).sleep();

  // wait until object appear in back_1 
  while(action_step != 2 && ros::Time::now().toSec() - go_straight_time_start < 2.0){
    if(saveFatalCollision())
      return false;
    ros::Duration(0.3).sleep();
  }
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Go ahead completed");
  ros::Duration(0.5).sleep();
  
  msg = turn_left();
  sendMotorCommand(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("End back left rotation completed");
  ros::Duration(0.5).sleep();

  action_step = 0;
  action_internal_condition = true;
  return true;
}
bool turn_front_left_plan(){
  geometry_msgs::Twist msg;
  
  msg = go_back();
  sendMotorCommand(msg);
  ros::Duration(0.25).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Back a little completed");
  ros::Duration(0.5).sleep();

  msg = turn_left();
  double rotate_time_start = ros::Time::now().toSec();
  sendMotorCommand(msg);
  
  // wait until front object disappear from front_2 or timeout
  while(action_step != 1 && ros::Time::now().toSec() - rotate_time_start < 3.0 ){
    if(saveFatalCollision())
      return false;
    action_internal_condition = false;    // switch next action_step
    ros::Duration(0.3).sleep();
  }
  double rotate_time = ros::Time::now().toSec() - rotate_time_start;
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("End first left rotation completed");
  ros::Duration(0.5).sleep();

  double go_straight_time_start = ros::Time::now().toSec();
  msg = go_straight();
  sendMotorCommand(msg);
  ros::Duration(0.5).sleep();

  // wait until object disappear in right_2 or timeout 
  while(action_step != 2 && ros::Time::now().toSec() - go_straight_time_start < 2.0){
    if(saveFatalCollision())
      return false;
    ros::Duration(0.3).sleep();
  }
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("Go ahead completed");
  ros::Duration(0.5).sleep();
  
  msg = turn_right();
  sendMotorCommand(msg);
  ros::Duration(rotate_time).sleep();
  msg = done();
  sendMotorCommand(msg);
  ROS_INFO("End back right rotation completed");
  ros::Duration(0.5).sleep();

  action_step = 0;
  action_internal_condition = true;
  return true;
}
bool go_back_plan(){
  geometry_msgs::Twist msg;
  
  msg = go_back();
  double back_time_start = ros::Time::now().toSec();
  sendMotorCommand(msg);
  
  // robot go back until an object appear in back_1 or back_2, or timeout 
  while(action_step == 0 && ros::Time::now().toSec() - back_time_start < 5){
    ros::Duration(1).sleep();
  }
  action_step = 0;
  msg = done();
  sendMotorCommand(msg);
  ros::Duration(1).sleep();

  // look right and left if possible to turn somewhere
  // left free
  if(get<0>(regions["left_1"]) > min_available_reg && get<0>(regions["left_2"]) > min_available_reg){
    // right free too
    if(get<0>(regions["left_1"]) > min_available_reg && get<0>(regions["left_2"]) > min_available_reg){
      // choose one with highest average
      if(get<1>(regions["right_1"]) + get<1>(regions["right_2"]) > get<1>(regions["left_1"]) + get<1>(regions["left_2"])){
        avoid_obj_plan_status = -3;
        ROS_INFO("Right and Left free -> take Right, which has highest average");
        turn_right_plan();
      }else{
        avoid_obj_plan_status = -4;
        ROS_INFO("Right and Left free -> take Left, which has highest average");
        turn_left_plan();
      }
    }else{  // right full, left free
      avoid_obj_plan_status = -4;
      ROS_INFO("Right full -> turn Left");
      turn_left_plan();
    }
  }else
    // left full, right free
    if(get<0>(regions["left_1"]) > min_available_reg && get<0>(regions["left_2"]) > min_available_reg){
       avoid_obj_plan_status = -3;
       ROS_INFO("Left full -> turn Right");
       turn_right_plan();
    }

  ROS_INFO("Back completed");
}
bool saveFatalCollision(){
  if(near_collision_local_ray[0] != -1 || near_collision_local_ray[1] != -1){
    sendMotorCommand(done());
    // check double near collision
    if(near_collision_local_ray[0] != -1 && near_collision_local_ray[1] != -1){  
      ROS_INFO("... how did you do that? ... hope DWAPlanner and GlobalPlanner will help you");
    }else 
      // near collision front -> go back!
      if(near_collision_local_ray[0] != -1){  
        ROS_INFO("NEAR COLLISION FRONT! Stop all and try to go back!");
        avoid_obj_plan_status = -5;
        go_back_plan();
    
    }else{
      // near collision back -> ... need to do something? for now, leave to planner!
        ROS_INFO("NEAR COLLISION BACK! Stop motors but leave to try to planner!");
    }
    near_collision_local_ray[0] = -1;
    near_collision_local_ray[1] = -1;
    return true;

  }else{
    return false;
  }
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
    if(i == 0 || i == 399 || (i >= 41 && i <= 43) || (i >= 110 && i <= 114) || (i >= 285 && i <= 289) || (i >= 356 && i <= 358))
      continue;
    
    if(choose_gate == -1){
      if (i < msg.ranges.size() * 3/8 && i >= msg.ranges.size() * 1/8){
        if(i < 85 && i >= 115)
          continue;
      }else if (i < msg.ranges.size() * 5/8 && i >= msg.ranges.size() * 3/8){
        if(i < 185 && i >= 215)
          continue;
      }
    }
    
    /** /
    if(msg.ranges.at(i) <= lethal_dist){
      cmd_pub.publish(done());
      mode = true;
      ROS_INFO("LETHAL DISTANCE OF RAY: %d !!! Interrupt all..", i);
      return;
    }
    /**/

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

        if ((i >= 0 && i < msg.ranges.size() * 1/8) || (i >= msg.ranges.size() * 7/8 && i < msg.ranges.size())){
          if(msg.ranges.at(i) <= lethal_dist_back && local_plan && mode){
            near_collision_local_ray[1] = i;  // Near collision back found
            ROS_INFO("%d RAY NEAR LETHAL COLLISION - val: %f",i,msg.ranges.at(i));
          }else
            near_collision_local_ray[1] = -1;
          back.push_back(msg.ranges.at(i));
        }else if (i < msg.ranges.size() * 3/8 && i >= msg.ranges.size() * 1/8){
          right.push_back(msg.ranges.at(i));
        }else if (i < msg.ranges.size() * 5/8 && i >= msg.ranges.size() * 3/8){
          if(msg.ranges.at(i) <= lethal_dist_front && local_plan && mode){
            near_collision_local_ray[0] = i;  // Near collision front found
          }else
            near_collision_local_ray[0] = -1;
          front.push_back(msg.ranges.at(i));
        }else if (i < msg.ranges.size() * 7/8 && i >= msg.ranges.size() * 5/8){
          left.push_back(msg.ranges.at(i));
        } 
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
    ROS_INFO("RIGHT_1 SIZE: %d - min, average: %f, %f", right.size(), get<0>(regions["right_1"]), get<1>(regions["right_1"]));
    ROS_INFO("RIGHT_2 SIZE: %d - min, average: %f, %f", right.size(), get<0>(regions["right_2"]), get<1>(regions["right_2"]));
    ROS_INFO("FRONT_1 SIZE: %d - min, average: %f, %f", front.size(), get<0>(regions["front_1"]), get<1>(regions["front_1"]));
    ROS_INFO("FRONT_2 SIZE: %d - min, average: %f, %f", front.size(), get<0>(regions["front_2"]), get<1>(regions["front_2"]));
    ROS_INFO("LEFT_1  SIZE: %d - min, average: %f, %f", left.size(), get<0>(regions["left_1"]), get<1>(regions["left_1"]));
    ROS_INFO("LEFT_2  SIZE: %d - min, average: %f, %f", left.size(), get<0>(regions["left_2"]), get<1>(regions["left_2"]));
    ROS_INFO("BACK_1  SIZE: %d - min, average: %f, %f", back.size(), get<0>(regions["back_1"]), get<1>(regions["back_1"]));
    ROS_INFO("BACK_2  SIZE: %d - min, average: %f, %f", back.size(), get<0>(regions["back_2"]), get<1>(regions["back_2"]));
    /**/
  
  if(!mode){
    if (!stop_laser)
      take_narrow_action();

  }else{
    
    if(!action_in_progress){  
      take_action();
    
    }else{
      switch (avoid_obj_plan_status){
        case -1:
          // rotate until obj disappear from front_2
          if(get<0>(regions["front_2"]) >= 1.5*min_obstacle_dist && action_internal_condition)
            action_step = 1;

          // check when obj enter in back_1
          //if(get<0>(regions["back_1"]) <= 1.5*min_obstacle_dist && !action_internal_condition)
          
          // check when obj exit from right_2
          if(get<0>(regions["right_2"]) >= min_obstacle_dist && !action_internal_condition)
            action_step = 2;
            
        break;
        case -2:
          // rotate until obj disappear from front_1
          if(get<0>(regions["front_1"]) >= 1.5*min_obstacle_dist && action_internal_condition)       
            action_step = 1;

          // check when obj enter in back_2
          //if(get<0>(regions["back_2"]) <= 1.5*min_obstacle_dist && !action_internal_condition)
          
          // check when obj exit from left_1
          if(get<0>(regions["left_1"]) >= min_obstacle_dist && !action_internal_condition)
            action_step = 2;

        break;
        case -3:
          // check front_1/front_2 free
          if(get<0>(regions["front_1"]) >= min_space_avail && get<0>(regions["front_2"]) >= min_space_avail)
            action_step = 1;
        break;
        case -4:
          // check front_1/front_2 free
          if(get<0>(regions["front_1"]) >= min_space_avail && get<0>(regions["front_2"]) >= min_space_avail)
            action_step = 1;
        break;
        case -5:
          // check back_1/back_2 until find object
          if(get<0>(regions["back_1"]) <= min_space_avail || get<0>(regions["back_2"]) <= min_space_avail)
            action_step = 1;
          
        break;
        case -6:
          // check if right platform free -> front_1 and front_2 <= 0.5
          if(get<0>(regions["front_1"]) <= find_obstacle && get<0>(regions["front_2"]) <= find_obstacle)
            free_right_platform++;
          
        break;
        case -7:
        break;
        default:
        break;
      }
    }
  }
}


/**** GOAL open space methods ****/
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
  goal.target_pose.header.stamp = ros::Time(0); /**/

  ROS_INFO("Goal pose (marrtino_map): [%f, %f, %f] - [%f, %f, %f, %f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
      goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, 
      goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);

  return goal;

}

void resultGoalsStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &goals_status_msgs){

  actionlib_msgs::GoalStatusArray goals_status_array = *goals_status_msgs;
  vector<actionlib_msgs::GoalStatus> goals_list = goals_status_array.status_list; 
  if(goals_list.size() != 0)
    goal_plan_status = goals_list.at(goals_list.size() - 1).status;
}

void correctPoseWRTOdom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose_msgs){
  MoveBaseClient ac("marrtino/move_base", true);

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
  
  // UNCOMMENT for move also the orientation 
  /*
  tf2::Quaternion q_current_goal, q_old_odom, q_odom, q_translation;
  tf2::fromMsg(current_goal_map_pose.pose.orientation, q_current_goal);
  tf2::fromMsg(current_amcl_pose.orientation, q_old_odom);
  tf2::fromMsg(amcl_pose.orientation, q_odom);
  q_translation = q_odom * q_old_odom.inverse();
  
  ROS_INFO("Relative traslation values: [%f, %f, %f] - [%f, %f, %f, %f]", amcl_pose.position.x - current_amcl_pose.position.x, 
            amcl_pose.position.y - current_amcl_pose.position.y, amcl_pose.position.z - current_amcl_pose.position.z, 
            q_translation.getX(), q_translation.getY(), q_translation.getZ(), q_translation.getW());

  
  current_goal_map_pose.pose.orientation = tf2::toMsg(q_translation * q_current_goal);
  */
  
  current_amcl_pose.position = amcl_pose.position;  
  current_amcl_pose.orientation = amcl_pose.orientation;

  current_goal_map_pose.header.frame_id = "marrtino_map";
  current_goal_map_pose.header.stamp = ros::Time(0); /**/

  /*
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
  current_goal_map_pose.header.stamp = ros::Time(0);
  */

  ROS_INFO("-> Goal pose update : [%f, %f, %f] - [%f, %f, %f, %f]", current_goal_map_pose.pose.position.x, current_goal_map_pose.pose.position.y,
      current_goal_map_pose.pose.position.z, current_goal_map_pose.pose.orientation.x, current_goal_map_pose.pose.orientation.y, 
      current_goal_map_pose.pose.orientation.z, current_goal_map_pose.pose.orientation.w);

  ac.sendGoal(getGoal(current_goal_map_pose));
  
}

void waitMarrtinoCallback(const std_msgs::String::ConstPtr& wait_msg){

  if(wait_msg->data == "loaded")
    obj_loaded = true;
  else
    obj_loaded = false;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  
  ros::Rate r(100);

  if (argc > 1){
    if(stoi(argv[1]) == 1){
      ROS_INFO("INFO: Node launched without interaction with ur_5!");
      hmw_4 = false;
    }else{
      hmw_4 = true;
    }

  }else{
    ROS_INFO("INFO: Node launched without interaction with ur_5!");
    hmw_4 = false;
  }
  
  initMap();

  //ros::Publisher cancel_goal = n.advertise<actionlib_msgs::GoalID>("/marrtino/move_base/cancel", 1);    ---> NOT WORK  
  //ros::Subscriber correct_goal_pose = n.subscribe("/marrtino/amcl_pose", 1, correctPoseWRTOdom);
  ros::Subscriber goals_status = n.subscribe("/marrtino/move_base/status", 1, resultGoalsStatus);
  ros::Subscriber sub_odom = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, odomPoseCallback);
  ros::Subscriber sub_laser = n.subscribe("/marrtino/scan", 1, laserReadCallback);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/move_base/cmd_vel", 1);
  initpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/marrtino/initialpose", 1);

  ros::Subscriber wait_marrtino_pub;
  if(hmw_4){
     wait_ur5_pub = n.advertise<std_msgs::String>("marrtino/wait_ur5", 1);
     wait_marrtino_pub = n.subscribe("/marrtino/wait_marrtino", 1, waitMarrtinoCallback);
  }
  
  MoveBaseClient ac("marrtino/move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal current_goal;
  current_goal_map_pose.header.frame_id = "marrtino_map";

  

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

    avoid_obj_plan_status = -1  # Pass to the right
                          = -2  # Pass to the left
                          = -3  # Turn right
                          = -4  # Turn left
                          = -5  # Go back
                          = -6  # find if there's object in right platform
                          = -10 # Not initialized yet

    global_narrow_state   = 0   # find_wall
                          = 1   # turn_right
                          = 2   # turn_left
                          = 3   # go_straight
                          = 4   # finish narrow mode -> change to open space mode
                          = -1  # Not initialized yet -> change_destination
    
    narrow_action_step    = 0   # go to end corridor
                          = 1   # go to gate 1
                          = 2   # go to gate 2
                          = 3   # go to left corner
                          = 4   # go to right corner
                          = 5   # rotate right steps
                          = 6   # rotate left steps
                          = 7   # rotate right
                          = 8   # rotate left
                          = 9   # go to end gate 1                      
                          
  /********************************/
  
  ros::AsyncSpinner spinner(0);

  while(ros::ok()){

    if(mode){

      if(!local_plan){                // if i'm planning in a local way don't take choice of global status
        switch (goal_plan_status){
          case -10:
            // State not initialized yet... wait
          break;
          case 0:
          
          break;
          case 1:
          
          break;
          case 2:
          
          break;
          case 3:
          {
            ROS_INFO("Goal plan %d Reached!", id_goals);
            action_in_progress = true;

            if(id_goals == 0){

              avoid_obj_plan_status = -6;
              
              // wait until check right platform busy -> to be sure wait at least 3 check from laser scan, with timeout of 5 sec 
              double start_waiting_time = ros::Time::now().toSec();
              while(free_right_platform < 3 && ros::Time::now().toSec() - start_waiting_time < 5){
                r.sleep();
              }

              if(free_right_platform < 3){
                ROS_INFO(" ----- Sending goal -----");
                // Send goal and start open space
                current_goal_map_pose.header.stamp = ros::Time(0); /**/
        
                // PLATFORM_RIGHT_GOAL --- GREEN final platform right
                current_goal_map_pose.pose.position.x = -0.517;//-0.482639;       //add 0.1 to center basket marrtino with platform
                current_goal_map_pose.pose.position.y = 0.5657;//0.565774;
                current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
                current_goal = getGoal(current_goal_map_pose);
                ac.sendGoal(current_goal);

                
              }else{
                /*
                local_plan = true;
                action_in_progress = true;
                avoid_obj_plan_status = -5;
                action_step = 0;
                geometry_msgs::Twist msg;
                msg = go_back();
                double back_time_start = ros::Time::now().toSec();
                sendMotorCommand(msg);
                
                // robot go back until an object appear in back_1 or back_2, or timeout 
                while(action_step == 0 && ros::Time::now().toSec() - back_time_start < 3){
                  ros::Duration(1).sleep();
                }
                action_step = 0;
                msg = done();
                sendMotorCommand(msg);
                ros::Duration(1).sleep();
                local_plan = false;
                action_in_progress = false;
                avoid_obj_plan_status = -10;
                */
                ROS_INFO(" ----- Sending goal -----");
                // Send goal and start open space
                current_goal_map_pose.header.stamp = ros::Time(0); /**/
        
                // PLATFORM_LEFT_GOAL --- GREEN final platform left
                current_goal_map_pose.pose.position.x = 0.2018; //0.101878;       //add 0.1 to center basket marrtino with platform
                current_goal_map_pose.pose.position.y = 0.557; //0.557094
                current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
                current_goal = getGoal(current_goal_map_pose);
                ac.sendGoal(current_goal);

                
              }
              id_goals++;
            
            }else if(id_goals == 1){
              
              // If hmw_4 is set -> wait interaction with ur_5 
              if(hmw_4){
                std_msgs::String wait_msgs;
                if(free_right_platform < 3){      // right platform
                  wait_msgs.data = "right";
                  wait_ur5_pub.publish(wait_msgs);
                }else{                            // left platform
                  wait_msgs.data = "left";
                  wait_ur5_pub.publish(wait_msgs);
                }
                ROS_INFO("Waiting that ur_5 load objects to marrtino..");

                while(!obj_loaded){
                  r.sleep();
                }
                obj_loaded = false;
                ROS_INFO("All object loaded!! Come back to home..");
              }

              ROS_INFO(" ----- Sending goal -----");
              // Send goal and start open space
              current_goal_map_pose.header.stamp = ros::Time(0); /**/
        
              // GATE 2
              current_goal_map_pose.pose.position.x = 1.243;
              current_goal_map_pose.pose.position.y = 2.415;
              current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);  
              current_goal = getGoal(current_goal_map_pose);
              ac.sendGoal(current_goal);
              
              free_right_platform = 0;
              id_goals++;
            
            }else{
              spinner.stop();
              mode = false;
              narrow_action_step = 3;
              global_narrow_state = -1;
              move_to_wall = 1;
            }
            ros::Duration(2).sleep();
            action_in_progress = false;
            avoid_obj_plan_status = -10;
          }
          break;
          case 4:
          {
            // Send goal and start open space
            current_goal_map_pose.header.stamp = ros::Time(0); /**/
        
            // come back to FIRST_GOAL 
            current_goal_map_pose.pose.position.x = -0.51;
            current_goal_map_pose.pose.position.y = 0.99;
            current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
            move_base_msgs::MoveBaseGoal current_goal = getGoal(current_goal_map_pose);
            ROS_INFO("___ Plan ABORTED!! come back to middle platform position and re-plan again ___");

            if(id_goals == 0){
              // Recovery action -> go back
              ROS_INFO("___ RECOVERY ACTION -> GO BACK ___");
              local_plan = true;
              action_in_progress = true;
              avoid_obj_plan_status = -5;
              go_back_plan();
              ROS_INFO(" ----- Sending goal -----");
              ac.sendGoal(current_goal);
            }else if(id_goals == 1 || id_goals == 2){
              ROS_INFO(" ----- Sending goal -----");
              ac.sendGoal(current_goal);
              id_goals--;
            }
            local_plan = false;
            action_in_progress = false;
            avoid_obj_plan_status = -10;
          }
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
      }

      switch (avoid_obj_plan_status){
        case -10:
          // State not initialized yet... wait
        break;
        case -1:
          local_plan = true;
          ac.cancelAllGoals();
          while(goal_plan_status != 2 && goal_plan_status != 8){
            r.sleep();
          }
          
          ROS_INFO("Goal plan Cancelled!");
          ROS_INFO("Start turn Front-Right!");
          turn_front_right_plan();
          ac.sendGoal(current_goal);
          
          local_plan = false;
          action_in_progress = false;
          avoid_obj_plan_status = -10;
        break;
        case -2:
          local_plan = true;
          ac.cancelAllGoals();
          while(goal_plan_status != 2 && goal_plan_status != 8){
            r.sleep();
          }
          
          ROS_INFO("Goal plan Cancelled!");
          ROS_INFO("Start turn Front-Left!");
          turn_front_left_plan();
          ac.sendGoal(current_goal);
          
          local_plan = false;
          action_in_progress = false;
          avoid_obj_plan_status = -10;
        break;
        case -3:
          local_plan = true;
          ac.cancelAllGoals();
          while(goal_plan_status != 2 && goal_plan_status != 8){
            r.sleep();
          }
          
          ROS_INFO("Goal plan Cancelled!");
          ROS_INFO("Start turn right!");
          turn_right_plan();
          ac.sendGoal(current_goal);
          
          action_in_progress = false;
          avoid_obj_plan_status = -10;
        break;
        case -4:
          local_plan = true;
          ac.cancelAllGoals();
          while(goal_plan_status != 2 && goal_plan_status != 8){
            r.sleep();
          }
          
          ROS_INFO("Goal plan Cancelled!");
          ROS_INFO("Start turn left!");
          turn_left_plan();
          ac.sendGoal(current_goal);
          
          local_plan = false;
          action_in_progress = false;
          avoid_obj_plan_status = -10;
        break;
        case -5:
          local_plan = true;
          ac.cancelAllGoals();
          while(goal_plan_status != 2 && goal_plan_status != 8){
            r.sleep();
          }
          
          ROS_INFO("Goal plan Cancelled!");
          go_back_plan();
          ac.sendGoal(current_goal);
          
          local_plan = false;
          action_in_progress = false;
          avoid_obj_plan_status = -10;
        break;
        default:
          ROS_INFO("Unknown state - avoid_obj_plan_status: %d!", avoid_obj_plan_status);
        break;
      }

    }else{
      geometry_msgs::Twist msg;
      if (global_narrow_state == 5 || finish_narrow_mode_check == 5){
        
        mode = true;
        sendMotorCommand(done());
        
        ROS_INFO(" ----- Sending goal -----");
        // Send goal and start open space
        current_goal_map_pose.header.stamp = ros::Time(0); /**/
        
        // FIRST_GOAL --- Before final platforms, in the middle 
        current_goal_map_pose.pose.position.x = -0.55;
        current_goal_map_pose.pose.position.y = 1.08;
        current_goal_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2 + 0.1);
        current_goal = getGoal(current_goal_map_pose);
        ac.sendGoal(current_goal);

        spinner.start();
        finish_narrow_mode_check = 0;
        id_goals = 0;
        action_in_progress = false;
        avoid_obj_plan_status = -10;


      }else if (global_narrow_state == 0)
          msg = find_wall_left();
      else if (global_narrow_state == 1)
        msg = turn_right();
      else if (global_narrow_state == 2)
        msg = turn_left();
      else if (global_narrow_state == 3)
          msg = go_straight();
      else if (global_narrow_state == 4)
          msg = find_wall_right();
      else if (global_narrow_state == -1){
        change_destination();
      }
      else{
        ROS_INFO("Unknown state!");
      }

      if(!mode)
        sendMotorCommand(msg);
    }    

    r.sleep();
  }

  ros::waitForShutdown();
  
  return 0;
}
