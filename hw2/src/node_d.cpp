#include "node_d.h"

using namespace std;

ros::Publisher gazebo_model_state_pub;
robot_model::RobotModelPtr kinematic_model;

map<string, int> frame_id_to_id;
map<int, string> id_to_gazebo_id;

vector<int> requested_objects;
vector<apriltag_ros::AprilTagDetection> found_objects;

int typeRun = 1; // 0 = from pcl of homework1_test; 1 = from apriltag
float extraZ = 0.3, space2rot = 0.01, tolerance = 0.01;

// Group to move
static const string PLANNING_GROUP = "manipulator";

// Interface for moving group
moveit::planning_interface::MoveGroupInterface *move_group;

// Planning scene to add and remove collision objects
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

// Used for valuate planning
//Eigen::Affine3d text_pose;

//Collision objects
vector<moveit_msgs::CollisionObject> collision_object_vector;
moveit_msgs::CollisionObject currentObject;
int id_obj_triang;

tf2::Quaternion q_gazebo, q_zero_ee;

bool find_gazebo_trian = false;            
bool triang = false, rotationZero = true;
bool processing = false;
bool attached = false;

void initializeMap(){
    frame_id_to_id.insert(pair<string, int>("red_cube_0", 0));
    frame_id_to_id.insert(pair<string, int>("red_cube_1", 1));
    frame_id_to_id.insert(pair<string, int>("red_cube_2", 2));
    frame_id_to_id.insert(pair<string, int>("red_cube_3", 3));
    frame_id_to_id.insert(pair<string, int>("yellow_cyl_0", 4));
    frame_id_to_id.insert(pair<string, int>("yellow_cyl_1", 5));
    frame_id_to_id.insert(pair<string, int>("green_prism_0", 6));
    frame_id_to_id.insert(pair<string, int>("green_prism_1", 7));
    frame_id_to_id.insert(pair<string, int>("green_prism_2", 8));
    frame_id_to_id.insert(pair<string, int>("blue_cube_0", 9));
    frame_id_to_id.insert(pair<string, int>("blue_cube_1", 10));
    frame_id_to_id.insert(pair<string, int>("blue_cube_2", 11));
    frame_id_to_id.insert(pair<string, int>("blue_cube_3", 12));
    frame_id_to_id.insert(pair<string, int>("red_prism_0", 13));
    frame_id_to_id.insert(pair<string, int>("red_prism_1", 14));
    frame_id_to_id.insert(pair<string, int>("red_prism_2", 15));

    id_to_gazebo_id.insert(pair<int, string>(0, "cube1"));
    id_to_gazebo_id.insert(pair<int, string>(1, "cube2"));
    id_to_gazebo_id.insert(pair<int, string>(2, "cube3"));
    id_to_gazebo_id.insert(pair<int, string>(3, "cube4"));
    id_to_gazebo_id.insert(pair<int, string>(4, "Hexagon0"));
    id_to_gazebo_id.insert(pair<int, string>(5, "Hexagon1"));
    id_to_gazebo_id.insert(pair<int, string>(6, "Triangle0"));
    id_to_gazebo_id.insert(pair<int, string>(7, "Triangle1"));
    id_to_gazebo_id.insert(pair<int, string>(8, "Triangle2"));
    id_to_gazebo_id.insert(pair<int, string>(9, "blue_cube_1"));
    id_to_gazebo_id.insert(pair<int, string>(10, "blue_cube_2"));
    id_to_gazebo_id.insert(pair<int, string>(11, "blue_cube_3"));
    id_to_gazebo_id.insert(pair<int, string>(12, "blue_cube_4"));
    id_to_gazebo_id.insert(pair<int, string>(13, "red_triangle_1"));
    id_to_gazebo_id.insert(pair<int, string>(14, "red_triangle_2"));
    id_to_gazebo_id.insert(pair<int, string>(15, "red_triangle_3"));
}

void startPosition(){
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //****************************//
    //          PLANNING          //
    //****************************//

    // Get joint names for group
    vector<string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);

    move_group->setNamedTarget("initial_configuration");
    ROS_INFO("Set initial target pose");
    // Try to plan movement from start to target position
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setPlanningTime(20.0);
    moveit::planning_interface::MoveItErrorCode success = move_group->plan(my_plan);

    // If a plan is found execute it
    if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
        throw runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    /****** VISUALIZATION ****** // 
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRemoteControl();
    //visual_tools.publishText(box_pose, "Over Object Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    /**/
    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    geometry_msgs::PoseStamped current_pose =
        move_group->getCurrentPose();

    ROS_INFO("Finish setInizialPosizion with pose of end effector:");
    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
             current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}

void endPosition(){
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //****************************//
    //          PLANNING          //
    //****************************//

    // Get joint names for group
    vector<string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);

    move_group->setNamedTarget("initial_configuration");
    ROS_INFO("Set initial target pose");
    // Try to plan movement from start to target position
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setPlanningTime(10.0);
    moveit::planning_interface::MoveItErrorCode success = move_group->plan(my_plan);

    // If a plan is found execute it
    if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
        throw runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    geometry_msgs::PoseStamped current_pose =
        move_group->getCurrentPose();

    ROS_INFO("Finish setInizialPosizion with pose of end effector:");
    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
             current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}

double to_degrees(double radians){
    return radians * (180.0 / M_PI);
}

geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll){      // yaw (Z), pitch (Y), roll (X)
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void moveOverObject(geometry_msgs::Pose box_pose, bool triang){

    move_group->setPoseReferenceFrame("world");

    //ROS_INFO("Pose Reference Frame: ", move_group.getPoseReferenceFrame());
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

    if(triang){
        tf2::Quaternion q_obj, q_current_ee, q_trasformed_ee;
        tf2::fromMsg(box_pose.orientation, q_obj);
        tf2::fromMsg(current_pose.pose.orientation, q_current_ee);
        q_trasformed_ee = q_obj * q_current_ee;
        box_pose.orientation = tf2::toMsg(q_trasformed_ee);
    }else{
        box_pose.orientation = current_pose.pose.orientation;
    }
    //****************************//
    //          PLANNING          //
    //****************************//

    // Get joint names for group
    vector<string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();
    
    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);
    move_group->setPoseTarget(box_pose, "ee_link");
    move_group->setGoalPositionTolerance(tolerance);
    move_group->setGoalOrientationTolerance(tolerance);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setPlanningTime(60.0);
    moveit::planning_interface::MoveItErrorCode success = move_group->plan(my_plan);

    // If a plan is found execute it
    if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
        throw runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    /****** VISUALIZATION ****** // 
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRemoteControl();
    visual_tools.deleteAllMarkers();
    //visual_tools.publishText(box_pose, "Over Object Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    /**/
    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    current_pose = move_group->getCurrentPose();

    ROS_INFO("Finish MoveOverObject with pose of end effector:");
    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
             current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}

void moveDown(){

    vector<geometry_msgs::Pose> waypoints;
    move_group->setPoseReferenceFrame("world");

    //Pose computed with respect to world frame
    geometry_msgs::Pose target_pose;
    geometry_msgs::PoseStamped current_pose =
        move_group->getCurrentPose();

    target_pose = current_pose.pose;
    target_pose.position.z = target_pose.position.z - extraZ + space2rot;
    waypoints.push_back(target_pose);
    ROS_INFO("Start Move down with coordinate:");
    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z,
             target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group->computeCartesianPath(waypoints,
                                                       0.01, // eef_step
                                                       0.0,  // jump_threshold
                                                       trajectory);

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;

    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->execute(my_plan);

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    current_pose = move_group->getCurrentPose();

    ROS_INFO("Finish MoveDown with pose of end effector:");
    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
             current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}

void correctTriangle(const gazebo_msgs::ModelStates &model_states){
    if(triang && !find_gazebo_trian){
        find_gazebo_trian = true;
        //ROS_INFO("Searching correct orientation of triangle");
        for(int i = 0; i < model_states.name.size(); i++){
            if(model_states.name[i] == id_to_gazebo_id.find(id_obj_triang)->second){
                geometry_msgs::Quaternion pose_gazebo = model_states.pose.at(i).orientation;
                tf2::fromMsg(pose_gazebo, q_gazebo);
                ROS_INFO("Original Triangle Orientation Found! [%f, %f, %f, %f]", q_gazebo.x(), q_gazebo.y(), q_gazebo.z(), q_gazebo.w());
                return;
            }
        }
        find_gazebo_trian = false;
    }
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_states_current){
    if(attached){
        moveit_msgs::AttachedCollisionObject attachedObj = planning_scene_interface->getAttachedObjects({currentObject.id}).at(currentObject.id);
        geometry_msgs::PoseStamped box_pose;
        box_pose.pose.position = attachedObj.object.primitive_poses.at(0).position;
        box_pose.pose.orientation.x = attachedObj.object.primitive_poses.at(0).orientation.x;
        box_pose.pose.orientation.y = attachedObj.object.primitive_poses.at(0).orientation.y;
        box_pose.pose.orientation.z = attachedObj.object.primitive_poses.at(0).orientation.z;
        box_pose.pose.orientation.w = attachedObj.object.primitive_poses.at(0).orientation.w;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseStamped target_pose_tf;

        ros::Duration timeout(0.1);
        try{
            transformStamped = tfBuffer.lookupTransform("world", attachedObj.object.header.frame_id, ros::Time(0), timeout);
            tf2::doTransform(box_pose, target_pose_tf, transformStamped);
        }
        catch (tf2::TransformException &ex){
            ROS_INFO("Error Trasformation...%s", ex.what());
        }

        if(triang){
            if(rotationZero){
                tf2::fromMsg(target_pose_tf.pose.orientation, q_zero_ee);
                rotationZero = false;
                return;
            }else{
                tf2::Quaternion q_attached, q_relative_rot;
                tf2::fromMsg(target_pose_tf.pose.orientation, q_attached);
                q_relative_rot = q_attached * q_zero_ee.inverse();
                q_gazebo = q_relative_rot * q_gazebo;
                target_pose_tf.pose.orientation = tf2::toMsg(q_gazebo); 

                //ROS_INFO("relative_rot = [%f, %f, %f, %f]", q_relative_rot.x(), q_relative_rot.y(), q_relative_rot.z(), q_relative_rot.w());
                q_zero_ee = q_attached;
            }
        }

        //ROS_INFO("Final pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", target_pose_tf.pose.position.x, target_pose_tf.pose.position.y, target_pose_tf.pose.position.z, 
        //                                target_pose_tf.pose.orientation.x, target_pose_tf.pose.orientation.y, target_pose_tf.pose.orientation.z, target_pose_tf.pose.orientation.w);

        gazebo_msgs::ModelState model_state;
        model_state.model_name = id_to_gazebo_id.find(stoi(currentObject.id))->second;
        model_state.pose = target_pose_tf.pose;
        model_state.twist.linear.x = 0.0;
        model_state.twist.linear.y = 0.0;
        model_state.twist.linear.z = 0.0;
        model_state.twist.angular.x = 0.0;
        model_state.twist.angular.y = 0.0;
        model_state.twist.angular.z = 0.0;
        model_state.reference_frame = string("world");

        gazebo_model_state_pub.publish(model_state);
    }
}

void tagDetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){

    if (!processing){
        processing = true;
        ROS_INFO("Message received");
        ROS_INFO("Objects detected: %d", msg->detections.size());

        // TODO: **** start cycle mean of poses

        for (int i = 0; i < msg->detections.size(); i++){
            apriltag_ros::AprilTagDetection message = msg->detections.at(i);

            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group->getPlanningFrame();
            collision_object.id = to_string(message.id.at(0)); // The id of the object is used to identify it.

            geometry_msgs::PoseStamped target_pose; // Pose computed with respect to world frame
            shape_msgs::SolidPrimitive primitive;   // Define a box to add to the world          
            
            float adjustTriangleBox = 0.0;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            if(typeRun){
                if(message.id.at(0) == 4 || message.id.at(0) == 5){           // Hexagon 0,1
                    primitive.dimensions[0] = 0.095;    // altezza // 0.09;
                    primitive.dimensions[1] = 0.095;     // raggio
                    primitive.dimensions[2] = 0.21;
                }else if(message.id.at(0) == 0 || message.id.at(0) == 1 ||    // CubeRed 0,1,2,3
                         message.id.at(0) == 2 || message.id.at(0) == 3 ||
                         message.id.at(0) == 9 || message.id.at(0) == 10 ||   // CubeBlue 0,1,2,3
                         message.id.at(0) == 11 || message.id.at(0) == 12){
                    primitive.dimensions[0] = 0.105;
                    primitive.dimensions[1] = 0.105;
                    primitive.dimensions[2] = 0.105;
                }else if(message.id.at(0) == 6 || message.id.at(0) == 7 ||    // GreenPrism 0,1,2
                         message.id.at(0) == 8 || message.id.at(0) == 13 ||   // RedPrism 0,1,2
                         message.id.at(0) == 14 || message.id.at(0) == 15){
                    primitive.dimensions[0] = 0.105;
                    primitive.dimensions[1] = 0.098;
                    primitive.dimensions[2] = 0.098;
                    adjustTriangleBox =  0.0336;   // (lato / 2) / sqrt(2);
                    //adjustTriangleBox = primitive.dimensions[2] / 2;
                }else{
                    ROS_INFO("Unknown object!!! can't process return..");
                    return;
                }
            }else{
                /** /
                if(message.id.at(0) == 4 || message.id.at(0) == 5){
                    primitive.dimensions[0] = 0.075;
                    primitive.dimensions[1] = 0.075;
                    primitive.dimensions[2] = 0.1;
                }else if(message.id.at(0) == 0 || message.id.at(0) == 1 ||
                         message.id.at(0) == 2 || message.id.at(0) == 3 ||
                         message.id.at(0) == 9 || message.id.at(0) == 10 ||
                         message.id.at(0) == 11 || message.id.at(0) == 12){
                    primitive.dimensions[0] = 0.105;
                    primitive.dimensions[1] = 0.105;
                    primitive.dimensions[2] = 0.105;
                }else if(message.id.at(0) == 6 || message.id.at(0) == 7 || 
                         message.id.at(0) == 8 || message.id.at(0) == 13 || 
                         message.id.at(0) == 14 || message.id.at(0) == 15){
                    primitive.dimensions[0] = 0.125;
                    primitive.dimensions[1] = 0.105;
                    primitive.dimensions[2] = 0.65;
                }else{
                    ROS_INFO("Unknown object!!! can't process return..");
                    return;
                }
                /**/
            }    

            target_pose.pose.orientation.x = message.pose.pose.pose.orientation.x;
            target_pose.pose.orientation.y = message.pose.pose.pose.orientation.y;
            target_pose.pose.orientation.z = message.pose.pose.pose.orientation.z;
            target_pose.pose.orientation.w = message.pose.pose.pose.orientation.w;
            target_pose.pose.position.x = message.pose.pose.pose.position.x;
            target_pose.pose.position.y = message.pose.pose.pose.position.y;
            target_pose.pose.position.z = message.pose.pose.pose.position.z;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::PoseStamped target_pose_tf;

            ros::Duration timeout(50.0);
            try{
                if (typeRun){
                    transformStamped = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time::now(), timeout);
                }
                else{
                    transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time::now(), timeout);
                }
                tf2::doTransform(target_pose, target_pose_tf, transformStamped);
            }
            catch (tf2::TransformException &ex){
                ROS_INFO("Error Trasformation...%s", ex.what());
            }

            // Define a pose for the box (specified relative to frame_id)
            geometry_msgs::Pose box_pose;
            box_pose.orientation.x = target_pose_tf.pose.orientation.x;
            box_pose.orientation.y = target_pose_tf.pose.orientation.y;
            box_pose.orientation.z = target_pose_tf.pose.orientation.z;
            box_pose.orientation.w = target_pose_tf.pose.orientation.w;
            box_pose.position.x = target_pose_tf.pose.position.x;
            box_pose.position.y = target_pose_tf.pose.position.y + adjustTriangleBox;
            box_pose.position.z = 0.87 + primitive.dimensions[2] / 2 - adjustTriangleBox;
            
            // TODO: **** end cycle mean of poses

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            collision_object_vector.push_back(collision_object);
            
            planning_scene_interface->applyCollisionObjects(collision_object_vector);

            ROS_INFO("Added into the world collision object:");
            ROS_INFO("\tid = %d : (rgb_optical_frame)", message.id.at(0));
            ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", target_pose_tf.pose.position.x, target_pose_tf.pose.position.y, target_pose_tf.pose.position.z,
                        target_pose_tf.pose.orientation.x, target_pose_tf.pose.orientation.y, target_pose_tf.pose.orientation.z, target_pose_tf.pose.orientation.w);
                
            for (int j = 0; j < requested_objects.size(); j++){
                if (message.id.at(0) == requested_objects.at(j)){
                    found_objects.push_back(message);
                }
            }
        }

        startPosition();
        for (int i = 0; i < found_objects.size(); i++){
            for (int j = 0; j < collision_object_vector.size(); j++){
                //ROS_INFO(" ***** TEST [i,j] -> %d, %d *****", found_objects.at(i).id.at(0), stoi(collision_object_vector.at(j).id));
                if (found_objects.at(i).id.at(0) == stoi(collision_object_vector.at(j).id)){
                    geometry_msgs::Pose target_pose = collision_object_vector.at(j).primitive_poses.at(0);
                    triang = false;
                    ROS_INFO("Move over object %d: ", found_objects.at(i).id.at(0));
                    ROS_INFO("\t\t- pose/orient = [%f, %f, %f] - [%f, %f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z,
                             target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
                    
                    if(collision_object_vector.at(j).id == "6" || collision_object_vector.at(j).id == "7" ||      // GreenPrism 0,1,2
                         collision_object_vector.at(j).id == "8" || collision_object_vector.at(j).id == "13" ||   // RedPrism 0,1,2
                         collision_object_vector.at(j).id == "14" || collision_object_vector.at(j).id == "15"){
                        id_obj_triang = stoi(collision_object_vector.at(j).id);
                        triang = true;
                        find_gazebo_trian = false;
                    }
                    
                    if(triang){
                        target_pose.position.z = target_pose.position.z + extraZ + 0.0336 + space2rot;
                        target_pose.position.y -= 0.0336;
                        moveOverObject(target_pose, true);
                        
                        moveDown();

                    }else{
                        target_pose.position.z = target_pose.position.z + collision_object_vector.at(j).primitives.at(0).dimensions[2] / 2 + extraZ + space2rot;
                        moveOverObject(target_pose, false);
                    
                        moveDown();
                    }
                    
                    ROS_INFO("Attach the object to the robot");
                    move_group->attachObject(collision_object_vector.at(j).id);
                    
                    ROS_INFO("Remove the object from the world");
                    planning_scene_interface->removeCollisionObjects({collision_object_vector.at(j).id});
                    //ros::Duration(0.1).sleep();

                    currentObject = collision_object_vector.at(j);
                    attached = true;
                    
                    // green final platform left
                    target_pose.position.x = 0.101878;
                    target_pose.position.y = 0.557094;
                    target_pose.position.z = 1.3;
                    // green final platform right
                    //target_pose.position.x = -0.482639;
                    //target_pose.position.y = 0.565774;
                    //target_pose.position.z = 1.2;
                    moveOverObject(target_pose, false);
                    
                    attached = false;
                    move_group->detachObject(collision_object_vector.at(j).id);
                    collision_object_vector.erase(collision_object_vector.begin() + j); 
                    ROS_INFO("Detauch object!");
                    startPosition();
                    break;
                }
            }
        }
        ROS_INFO("Remove the remaining objects added from the world");
        for (int j = 0; j < collision_object_vector.size(); j++)
            if(stoi(collision_object_vector.at(j).id) != 100)       // don't remove table
                planning_scene_interface->removeCollisionObjects({collision_object_vector.at(j).id});
        processing = false;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "node_d");
    ros::NodeHandle n; //, n1, n2;    

    //	ROS_INFO("argc: %d", argc);
    if (argc < 2){
        ROS_INFO("Usage: rosrun hw_2 node_d [0 = pcl, 1+ = apriltag] frame_id_1 frame_id_2 ...");
        return 0;
    }
    for (int i = 0; i < argc; i++)
        ROS_INFO("argv %d = %s", i, argv[i]);

    initializeMap();
    ROS_INFO("Map initialized");

    typeRun = atoi(argv[1]);
    for (int i = 2; i < argc; i++){
        ROS_INFO("Object requested: %s", argv[i]);
        requested_objects.push_back(frame_id_to_id.at(argv[i]));
    }

    /****** VISUALIZATION ****** // 
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRemoteControl();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    /**/

    // ****** SETUP ****** //
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Clear previus collison objects in the scene and attached in the robot, if there are
    // Detach an object from the robot if there is
    ROS_INFO("Try removing attached objects from robot! Waiting (about 3 sec)...");
    for(int i = 0; i < 16; i++){
        move_group->detachObject(to_string(i));
        ros::Duration(0.1).sleep();
    }

    // Remove all object from the world if there are
    ROS_INFO("Try removing collision objects from world!");
    planning_scene_interface->getAttachedObjects();
    vector<string> previoysObjectNames = planning_scene_interface->getKnownObjectNames();
    for (int j = 0; j < previoysObjectNames.size(); j++){       // all our objects added
        planning_scene_interface->removeCollisionObjects({previoysObjectNames.at(j)});
        ROS_INFO("Object %d removed from planning scene", stoi(previoysObjectNames.at(j)));
    }
    

    // Add collision object of table
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.785;
    primitive.dimensions[2] = 0.27;
    geometry_msgs::Pose table_pose;
    tf2::Quaternion q_table;
    q_table.setRPY(0, 0, 0);
    table_pose.orientation = tf2::toMsg(q_table);
    table_pose.position.x = 0.0;
    table_pose.position.y = -0.5925;
    table_pose.position.z = 0.735;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "100";              
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    collision_object_vector.push_back(collision_object);
    planning_scene_interface->applyCollisionObjects(collision_object_vector);

    // ****** Publish and Subscribe ****** //
    ros::Subscriber sub;
    if (typeRun){
        sub = n.subscribe("/tag_detections", 1, tagDetectCallback);
        ROS_INFO("Node started and subscribed to /tag_detections");
    }
    else{
        sub = n.subscribe("/pose_objects", 1, tagDetectCallback);
        ROS_INFO("Node started and subscribed to /pose_objects");
    }
    
    ros::Subscriber joint_states_sub = n.subscribe("/ur5/joint_states", 1, jointStatesCallback);
    ros::Subscriber triangle_sub = n.subscribe("/gazebo/model_states", 1, correctTriangle);

    gazebo_model_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    ros::AsyncSpinner async_spinner_glob(0);
    async_spinner_glob.start();

    ros::waitForShutdown();
    return 0;
}
