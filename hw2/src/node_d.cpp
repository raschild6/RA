#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

map<string, int> frame_id_to_id;
vector<int> requested_objects;
vector<apriltag_ros::AprilTagDetection> found_objects;

ofstream myfile;

// Group to move
static const std::string PLANNING_GROUP = "manipulator";

// Interface for moving group
moveit::planning_interface::MoveGroupInterface *move_group;

// Planning scene to add and remove collision objects
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

//Collision objects
std::vector<moveit_msgs::CollisionObject> collision_objects;

bool processing = false;
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

    // PLANNING
    // ^^^^^^^^^^^^^^^^^^^^^^^^^

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
}

double to_degrees(double radians) {
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
void moveOverObject(apriltag_ros::AprilTagDetection object)
{

    //Pose computed with respect to world frame
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = ToQuaternion(to_degrees(1.57), to_degrees(0.13), to_degrees(-1.57));
    //target_pose1.orientation.x = 0;
    //target_pose1.orientation.y = 0;
    //target_pose1.orientation.z = -1.57;
    //target_pose1.orientation.w = 0;
    target_pose1.position.x = object.pose.pose.pose.position.x;
    target_pose1.position.y = object.pose.pose.pose.position.y;
    target_pose1.position.z = object.pose.pose.pose.position.z - 0.5;
    move_group->setPoseReferenceFrame("world");

    ROS_INFO("target pose of object:");
    ROS_INFO("\t\t- pose = [%f, %f, %f]", target_pose1.position.x,
             target_pose1.position.y,
             target_pose1.position.z);
    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", target_pose1.orientation.x,
             target_pose1.orientation.y,
             target_pose1.orientation.z,
             target_pose1.orientation.w);

    //ROS_INFO("Pose Reference Frame: ", move_group.getPoseReferenceFrame());
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // PLANNING
    // ^^^^^^^^^^^^^^^^^^^^^^^^^

    // Get joint names for group
    std::vector<std::string> manipulator_joint_names;
    manipulator_joint_names = move_group->getJoints();

    // Set start state to the current robot state
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(1.0);

    move_group->setPoseTarget(target_pose1, "ee_link");
    move_group->setGoalPositionTolerance(0.05);
    move_group->setGoalOrientationTolerance(0.05);
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
            primitive.dimensions[0] = 0.065;
            primitive.dimensions[1] = 0.065;
            primitive.dimensions[2] = 0.095;

            // Define a pose for the box (specified relative to frame_id)
            geometry_msgs::Pose box_pose;
            box_pose.orientation.x = message.pose.pose.pose.orientation.x;
            box_pose.orientation.y = message.pose.pose.pose.orientation.y;
            box_pose.orientation.z = message.pose.pose.pose.orientation.z;
            box_pose.orientation.w = message.pose.pose.pose.orientation.w;
            box_pose.position.x = message.pose.pose.pose.position.x;
            box_pose.position.y = message.pose.pose.pose.position.y;
            //box_pose.position.z = message.pose.pose.pose.position.z;
            box_pose.position.z = 0.735 + primitive.dimensions[2] / 2;
            ;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);

            ROS_INFO("Added collision object %s into the world", collision_object.id);
            planning_scene_interface->addCollisionObjects(collision_objects);

            for (int j = 0; j < requested_objects.size(); j++)
            {
                if (message.id.at(0) == requested_objects.at(j))
                {
                    found_objects.push_back(message);

                    ROS_INFO("id = %d :", message.id.at(0));
                    ROS_INFO("\t\t- pose = [%f, %f, %f]", message.pose.pose.pose.position.x,
                             message.pose.pose.pose.position.y,
                             message.pose.pose.pose.position.z);
                    ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", message.pose.pose.pose.orientation.x,
                             message.pose.pose.pose.orientation.y,
                             message.pose.pose.pose.orientation.z,
                             message.pose.pose.pose.orientation.w);
                }
            }
        }
        //ros::Duration(2.0).sleep();
        for (int i = 0; i < found_objects.size(); i++)
        {
            for (int j = 0; j < collision_objects.size(); j++)
            {
                if (found_objects.at(i).id.at(0) == collision_objects.at(j).id.at(0))
                {
                    moveOverObject(found_objects.at(i));

                    collision_objects.erase(collision_objects.begin() + j);
                }
            }
        }
        processing = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_d");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //	ROS_INFO("argc: %d", argc);
    if (argc < 1)
    {
        ROS_INFO("Usage: rosrun hw_2 node_d frame_id_1 frame_id_2 ...");
    }

    initializeMap();
    ROS_INFO("Map initialized");

    for (int i = 1; i < argc; i++)
    {
        ROS_INFO("Object requested: %s", argv[i]);
        requested_objects.insert(requested_objects.begin(), frame_id_to_id.at(argv[i]));
    }

    myfile.open("detections.txt");

    // SETUP
    // --------------
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;

    startPosition();
    ros::Subscriber sub = n.subscribe("/tag_detections", 1000, chatterCallback);
    ROS_INFO("Node started and subscribed to /tag_detections");

    myfile.close();
    ros::waitForShutdown();
    return 0;
}
