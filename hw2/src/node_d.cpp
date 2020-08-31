#include "node_d.h"

using namespace std;
ros::Publisher gazebo_model_state_pub;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;

map<string, int> frame_id_to_id;
vector<int> requested_objects;
vector<apriltag_ros::AprilTagDetection> found_objects;

ofstream myfile;

int typeRun = 1;        // 0 = from pcl of homework1_test; 1 = from apriltag

// Group to move
static const std::string PLANNING_GROUP = "manipulator";

// Interface for moving group
moveit::planning_interface::MoveGroupInterface *move_group;

// Planning scene to add and remove collision objects
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

//Collision objects
std::vector<moveit_msgs::CollisionObject> collision_objects;

bool processing = false;
bool attached = false;
void initializeMap()
{
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
}

void startPosition()
{
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
    //****************************//
    //          PLANNING          //
    //****************************//

    // Get joint names for group
    std::vector<std::string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);

    move_group->setNamedTarget("initial_configuration");
    ROS_INFO("Set initial target pose");
    // Try to plan movement from start to target position
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setPlanningTime(5.0);
    moveit::planning_interface::MoveItErrorCode success = move_group->plan(my_plan);

    // If a plan is found execute it
    if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
        throw std::runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    geometry_msgs::PoseStamped current_pose =
        move_group->getCurrentPose();

    ROS_INFO("pose of end effector:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", current_pose.pose.orientation.x,
             current_pose.pose.orientation.y,
             current_pose.pose.orientation.z,
             current_pose.pose.orientation.w);
}

double to_degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
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
void moveOverObject(apriltag_ros::AprilTagDetection object, geometry_msgs::Pose box_pose)
{
    move_group->setPoseReferenceFrame("world");

    //ROS_INFO("Pose Reference Frame: ", move_group.getPoseReferenceFrame());
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

    box_pose.orientation = current_pose.pose.orientation;
    //****************************//
    //          PLANNING          //
    //****************************//
    
    // Get joint names for group
    std::vector<std::string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);
    ROS_INFO("Move over:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", box_pose.position.x,
             box_pose.position.y,
             box_pose.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", box_pose.orientation.x,
             box_pose.orientation.y,
             box_pose.orientation.z,
             box_pose.orientation.w);

    move_group->setPoseTarget(box_pose, "ee_link");
    move_group->setGoalPositionTolerance(0.05);
    move_group->setGoalOrientationTolerance(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->setPlanningTime(20.0);
    moveit::planning_interface::MoveItErrorCode success = move_group->plan(my_plan);

    // If a plan is found execute it
    if (success != moveit_msgs::MoveItErrorCodes::SUCCESS)
        throw std::runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    current_pose = move_group->getCurrentPose();

    ROS_INFO("pose of end effector:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", current_pose.pose.orientation.x,
             current_pose.pose.orientation.y,
             current_pose.pose.orientation.z,
             current_pose.pose.orientation.w);
}
void moveDown()
{
    std::vector<geometry_msgs::Pose> waypoints;
    move_group->setPoseReferenceFrame("world");

    //Pose computed with respect to world frame
    geometry_msgs::Pose target_pose;
    geometry_msgs::PoseStamped current_pose =
        move_group->getCurrentPose();

    target_pose = current_pose.pose;
    target_pose.position.z = 0.735 + 0.270 + 0.08;
    waypoints.push_back(target_pose);
    ROS_INFO("Move down:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", target_pose.orientation.x,
             target_pose.orientation.y,
             target_pose.orientation.z,
             target_pose.orientation.w);

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

    ROS_INFO("pose of end effector:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", current_pose.pose.orientation.x,
             current_pose.pose.orientation.y,
             current_pose.pose.orientation.z,
             current_pose.pose.orientation.w);
}

void chatterCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{

    if (!processing)
    {
        processing = true;
        ROS_INFO("Message received");
        ROS_INFO("Objects detected: %d", msg->detections.size());
        //	for (int i = 0; i < requested_objects.size(); i++)
        //	{
        //		ROS_INFO("ID object requested: %d", requested_objects.at(i));
        //	}
        for (int i = 0; i < msg->detections.size(); i++)
        {
            apriltag_ros::AprilTagDetection message = msg->detections.at(i);

            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group->getPlanningFrame();
            // The id of the object is used to identify it.
            collision_object.id = message.id.at(0);
            // Define a box to add to the world.
            //Dimensioni diverse in base al tipo di oggetto (cilindro)
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.075;
            primitive.dimensions[1] = 0.075;
            primitive.dimensions[2] = 0.1;
            /* Exagon Exact Apriltag
            primitive.dimensions[0] = 0.065;
            primitive.dimensions[1] = 0.065;
            primitive.dimensions[2] = 0.095;
            */

           //Pose computed with respect to world frame
            geometry_msgs::PoseStamped target_pose;
            //geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

            //target_pose1.orientation = ToQuaternion(to_degrees(1.57), to_degrees(0), to_degrees(1.57));
            target_pose.pose.orientation.x = 0.00;
            target_pose.pose.orientation.y = 0.00;
            target_pose.pose.orientation.z = 0.00;
            target_pose.pose.orientation.w = 1.00;
            target_pose.pose.position.x = message.pose.pose.pose.position.x;
            target_pose.pose.position.y = message.pose.pose.pose.position.y;
            target_pose.pose.position.z = message.pose.pose.pose.position.z;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::PoseStamped target_pose_tf;

            ros::Duration timeout(50.0);
            try{ 
                if(typeRun){
                //camera_rgb_optical_frame
                transformStamped = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time::now(), timeout);
                //transformStamped.header.frame_id = "world";
                }else{
                    transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time::now(), timeout);
                }
                tf2::doTransform(target_pose, target_pose_tf, transformStamped);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_INFO("Error Trasformation...%s", ex.what());
            }

            ROS_INFO("Move over object:");
            ROS_INFO("\t\t- pose = [%f, %f, %f]", target_pose_tf.pose.position.x,
                    target_pose_tf.pose.position.y,
                    target_pose_tf.pose.position.z);


            // Define a pose for the box (specified relative to frame_id)
            geometry_msgs::Pose box_pose;
            box_pose.orientation.x = target_pose_tf.pose.orientation.x;
            box_pose.orientation.y = target_pose_tf.pose.orientation.y;
            box_pose.orientation.z = target_pose_tf.pose.orientation.z;
            box_pose.orientation.w = target_pose_tf.pose.orientation.w;
            box_pose.position.x = target_pose_tf.pose.position.x;
            box_pose.position.y = target_pose_tf.pose.position.y;
            box_pose.position.z = 1.005 + primitive.dimensions[2] / 2;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);

            ROS_INFO("Added collision object %d into the world", collision_object.id);
            planning_scene_interface->addCollisionObjects(collision_objects);

            for (int j = 0; j < requested_objects.size(); j++)
            {
                if (message.id.at(0) == requested_objects.at(j))
                {
                    found_objects.push_back(message);

                    ROS_INFO("id = %d : (rgb_optical_frame)", message.id.at(0));
                    ROS_INFO("\t\t- pose = [%f, %f, %f]", target_pose_tf.pose.position.x,
                                                            target_pose_tf.pose.position.y,
                                                            target_pose_tf.pose.position.z);
                    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", target_pose_tf.pose.orientation.x,
                                                                    target_pose_tf.pose.orientation.y,
                                                                    target_pose_tf.pose.orientation.z,
                                                                    target_pose_tf.pose.orientation.w);
                }
            }
        }
        startPosition();
        //ros::Duration(2.0).sleep();
        for (int i = 0; i < found_objects.size(); i++)
        {
            for (int j = 0; j < collision_objects.size(); j++)
            {
                if (found_objects.at(i).id.at(0) == collision_objects.at(j).id.at(0))
                {
                    geometry_msgs::Pose target_pose = collision_objects.at(j).primitive_poses.at(0);
                    target_pose.position.z = target_pose.position.z + collision_objects.at(j).primitives.at(0).dimensions[2] / 2 + 0.825;
                    moveOverObject(found_objects.at(i), target_pose);

                    moveDown();

                    ROS_INFO("Attach the object to the robot");
                    move_group->attachObject(collision_objects.at(j).id);

                    std::vector<std::string> object_ids;
                    object_ids.push_back(collision_objects.at(j).id);
                    planning_scene_interface->removeCollisionObjects(object_ids);
                    collision_objects.erase(collision_objects.begin() + j);
                    attached = true;
                    ros::Duration(1.0).sleep();
                    startPosition();
                }
            }
        }
        processing = false;
    }
}
void jointStatesCallback(const sensor_msgs::JointState &joint_states_current)
{
    if (attached)
    {
        /**/
        const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
        const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

        std::vector<double> joint_states;
        for (size_t i = 0; i < joint_states_current.position.size() - 2; ++i)
        {
            joint_states.push_back(joint_states_current.position[i + 2]);
        }
        kinematic_state->setJointGroupPositions(joint_model_group, joint_states);

        const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");
        ROS_INFO("Link pose: [%f, %f, %f]", end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z());

        //double end_effector_z_offset = 0.125;
        //Eigen::Affine3d tmp_transform(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, end_effector_z_offset)));

        //Eigen::Affine3d newState = end_effector_state * tmp_transform;
        Eigen::Affine3d newState = end_effector_state;
        
        /** /
        move_group->setPoseReferenceFrame("world");
        geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
        /**/
        /*
        ROS_INFO("pose of end effector:");
        ROS_INFO("\t\t- pose = [%f, %f, %f]", current_pose.pose.position.x,
                 current_pose.pose.position.y,
                 current_pose.pose.position.z);
        ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", current_pose.pose.orientation.x,
                 current_pose.pose.orientation.y,
                 current_pose.pose.orientation.z,
                 current_pose.pose.orientation.w);
        geometry_msgs::Pose pose;
        pose.position.x = current_pose.pose.position.x;
        pose.position.y = current_pose.pose.position.y;
        pose.position.z = current_pose.pose.position.z - 0.1;
        */
        geometry_msgs::Pose pose;
        pose.position.x = end_effector_state.translation().x();
        pose.position.y = end_effector_state.translation().y();
        pose.position.z = end_effector_state.translation().z() - 0.1;

        /*
        Eigen::Quaterniond quat(newState.rotation());
        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        */

        gazebo_msgs::ModelState model_state;
        // This string results from the spawn_urdf call in the box.launch file argument: -model box
        model_state.model_name = std::string("Hexagon0");
        model_state.pose = pose;
        model_state.reference_frame = std::string("world");

        ROS_INFO("Hexagon pose: [%f, %f, %f]", pose.position.x, pose.position.y, pose.position.z);
        gazebo_model_state_pub.publish(model_state);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_d");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //	ROS_INFO("argc: %d", argc); 
    if (argc < 1){
        ROS_INFO("Usage: rosrun hw_2 node_d [0 = pcl, 1+ = apriltag] frame_id_1 frame_id_2 ...");
    }
    for (int i = 0; i < argc; i++)
        ROS_INFO("argv %d = %s", i, argv[i]); 
    
    initializeMap();
    ROS_INFO("Map initialized");
    
    typeRun = atoi(argv[1]);
    for (int i = 2; i < argc; i++)
    {
        ROS_INFO("Object requested: %s", argv[i]);
        requested_objects.insert(requested_objects.begin(), frame_id_to_id.at(argv[i]));

    }

    // SETUP
    // --------------
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

    ros::Subscriber sub;
    if(typeRun){
        sub = n.subscribe("/tag_detections", 1000, chatterCallback); 
        ROS_INFO("Node started and subscribed to /tag_detections");
    }else{
        sub = n.subscribe("/pose_objects", 1000, chatterCallback);
        ROS_INFO("Node started and subscribed to /pose_objects");
    }
    ros::Subscriber joint_states_sub = n.subscribe("/ur5/joint_states", 1, jointStatesCallback);
    gazebo_model_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    ros::waitForShutdown();
    return 0;
}
