#include "follow_wall.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <string>
#include <numeric>
using namespace std;

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

ros::Publisher cmd_pub;
float range_min = 0;
float range_max = 0;
unordered_map<string, tuple<float, float>> regions; // name_region, < min_value, mean_value >
int global_state = 0;
int move_to_wall = 1;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
geometry_msgs::PoseStamped des_pose;
double yaw = 0;
double yaw_precision = M_PI / (90 * 2); // +/- 1 degree allowed
double dist_precision = 0.1;

void initMap()
{
    // each of 4 regions has a first part(1) and next part(2) of rays
    regions["right_1"] = tuple<float, float>(-1, -1);
    regions["right_2"] = tuple<float, float>(-1, -1);
    regions["front_1"] = tuple<float, float>(-1, -1);
    regions["front_2"] = tuple<float, float>(-1, -1);
    regions["left_1"] = tuple<float, float>(-1, -1);
    regions["left_2"] = tuple<float, float>(-1, -1);
    regions["back_1"] = tuple<float, float>(-1, -1);
    regions["back_2"] = tuple<float, float>(-1, -1);
}

void change_state(int state)
{
    ROS_INFO("CHANGE_STATE %d", state);
    if (state != global_state)
    {
        //ROS_INFO("Wall follower - [%s]", state);
        global_state = state;
    }
}
void take_action()
{
    //ROS_INFO("TAKE ACTION");
    string state_description = "";

    //float d = 0.17;  //good for min value
    float d_front = 0.3;
    float d_back = 0.1;

    if (get<1>(regions["left_1"]) < d_front && get<1>(regions["right_2"]) > d_front)
    {
        /*
        if (get<1>(regions["left_2"]) < d_back || get<1>(regions["right_1"]) > d_back)
        {
            ROS_INFO("TAKE ACTION \t\t GO STRAIGHT BACK");
            move_to_wall = 0;
            state_description = "case 4 - nothing";
            change_state(3); //go straight forward
        }
        else
        {
        */
            ROS_INFO("TAKE ACTION \t\t TURN RIGHT");

            move_to_wall = 0;
            state_description = "case 2 - left";
            change_state(1); //turn right
        //}
    }
    else if (get<1>(regions["left_1"]) > d_front && get<1>(regions["right_2"]) < d_front)
    {
        /*if (get<1>(regions["left_2"]) < d_back || get<1>(regions["right_1"]) > d_back)
        {
            ROS_INFO("TAKE ACTION \t\t GO STRAIGHT BACK");
            move_to_wall = 0;
            state_description = "case 4 - nothing";
            change_state(3); //go straight forward
        }
        else
        {
        */
            ROS_INFO("TAKE ACTION \t\t TURN LEFT");

            move_to_wall = 0;
            state_description = "case 3 - right";
            change_state(2); //turn left
        //}
    }
    else if (get<1>(regions["left_1"]) > d_front && get<1>(regions["right_2"]) > d_front)
    {
        if (move_to_wall)
        {
            ROS_INFO("TAKE ACTION \t\t TURN LEFT TO WALL");

            state_description = "case 1 - nothing";
            change_state(0); //find wall by rotating left and going forward
        }
        else
        {
            ROS_INFO("TAKE ACTION \t\t GO STRAIGHT");

            state_description = "case 4 - nothing";
            change_state(3); //go straight forward
        }
    }
}

geometry_msgs::Twist find_wall()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = 0.1;
    return msg;
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
geometry_msgs::Twist go_straight()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    return msg;
}
void laserReadCallback(const sensor_msgs::LaserScan &msg)
{
    ROS_INFO("LASER READ");
    range_min = msg.range_min;
    range_max = msg.range_max;
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    vector<float> right, front, left, back = {};
    ROS_INFO("LASER RAYS: %d", msg.ranges.size());
    for (int i = 0; i < msg.ranges.size(); i++)
    {

        //ROS_INFO("msg at %d: %f", i, msg.ranges.at(i));

        // Skip robot stake ray
        if (i == 0 || (i >= 41 && i <= 43) || (i >= 110 && i <= 114) || (i >= 285 && i <= 289) || (i >= 356 && i <= 358))
            continue;

        if (msg.ranges.at(i) > range_min)
        {
            if (msg.ranges.at(i) > 12.0)
            {
                if ((i >= 0 && i < msg.ranges.size() * 1 / 8) || (i >= msg.ranges.size() * 7 / 8 && i < msg.ranges.size()))
                    back.push_back(12.0);
                else if (i < msg.ranges.size() * 3 / 8 && i >= msg.ranges.size() * 1 / 8)
                    right.push_back(12.0);
                else if (i < msg.ranges.size() * 5 / 8 && i >= msg.ranges.size() * 3 / 8)
                    front.push_back(12.0);
                else if (i < msg.ranges.size() * 7 / 8 && i >= msg.ranges.size() * 5 / 8)
                    left.push_back(12.0);
            }
            else
            {

                if ((i >= 0 && i < msg.ranges.size() * 1 / 8) || (i >= msg.ranges.size() * 7 / 8 && i < msg.ranges.size()))
                    back.push_back(msg.ranges.at(i));
                else if (i < msg.ranges.size() * 3 / 8 && i >= msg.ranges.size() * 1 / 8)
                    right.push_back(msg.ranges.at(i));
                else if (i < msg.ranges.size() * 5 / 8 && i >= msg.ranges.size() * 3 / 8)
                    front.push_back(msg.ranges.at(i));
                else if (i < msg.ranges.size() * 7 / 8 && i >= msg.ranges.size() * 5 / 8)
                    left.push_back(msg.ranges.at(i));
            }
        }
    }

    regions["right_1"] = tuple<float, float>(*min_element(begin(right), begin(right) + (right.size() / 2) - 1), accumulate(begin(right), begin(right) + (right.size() / 2) - 1, 0.0) / (right.size() / 2));
    regions["right_2"] = tuple<float, float>(*min_element(begin(right) + right.size() / 2, end(right)), accumulate(begin(right) + right.size() / 2, end(right), 0.0) / (right.size() / 2));
    regions["front_1"] = tuple<float, float>(*min_element(begin(front), begin(front) + (front.size() / 2) - 1), accumulate(begin(front), begin(front) + (front.size() / 2) - 1, 0.0) / (front.size() / 2));
    regions["front_2"] = tuple<float, float>(*min_element(begin(front) + front.size() / 2, end(front)), accumulate(begin(front) + front.size() / 2, end(front), 0.0) / (front.size() / 2));
    regions["left_1"] = tuple<float, float>(*min_element(begin(left), begin(left) + (left.size() / 2) - 1), accumulate(begin(left), begin(left) + (left.size() / 2) - 1, 0.0) / (left.size() / 2));
    regions["left_2"] = tuple<float, float>(*min_element(begin(left) + left.size() / 2, end(left)), accumulate(begin(left) + left.size() / 2, end(left), 0.0) / (left.size() / 2));
    regions["back_2"] = tuple<float, float>(*min_element(begin(back), begin(back) + (back.size() / 2) - 1), accumulate(begin(back), begin(back) + (back.size() / 2) - 1, 0.0) / (back.size() / 2));
    regions["back_1"] = tuple<float, float>(*min_element(begin(back) + back.size() / 2, end(back)), accumulate(begin(back) + back.size() / 2, end(back), 0.0) / (back.size() / 2));
    // NB. back is inverted obv.

    ROS_INFO("RIGHT_1 SIZE: %d - min, mean: %f, %f", right.size(), get<0>(regions["right_1"]), get<1>(regions["right_1"]));
    ROS_INFO("RIGHT_2 SIZE: %d - min, mean: %f, %f", right.size(), get<0>(regions["right_2"]), get<1>(regions["right_2"]));
    ROS_INFO("FRONT_1 SIZE: %d - min, mean: %f, %f", front.size(), get<0>(regions["front_1"]), get<1>(regions["front_1"]));
    ROS_INFO("FRONT_2 SIZE: %d - min, mean: %f, %f", front.size(), get<0>(regions["front_2"]), get<1>(regions["front_2"]));
    ROS_INFO("LEFT_1  SIZE: %d - min, mean: %f, %f", left.size(), get<0>(regions["left_1"]), get<1>(regions["left_1"]));
    ROS_INFO("LEFT_2  SIZE: %d - min, mean: %f, %f", left.size(), get<0>(regions["left_2"]), get<1>(regions["left_2"]));
    ROS_INFO("BACK_1  SIZE: %d - min, mean: %f, %f", back.size(), get<0>(regions["back_1"]), get<1>(regions["back_1"]));
    ROS_INFO("BACK_2  SIZE: %d - min, mean: %f, %f", back.size(), get<0>(regions["back_2"]), get<1>(regions["back_2"]));

    take_action();
}

void done()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    global_state = -1;
    ROS_INFO("GOAL REACHED");
    cmd_pub.publish(twist_msg);
}

void check_goal()
{
    double desired_yaw = atan2(des_pose.pose.position.y - robot_pose.pose.pose.position.y, des_pose.pose.position.x - robot_pose.pose.pose.position.x);
    //ROS_INFO("DESIRED YAW: %f", desired_yaw);
    double err_yaw = desired_yaw - yaw;
    double err_pos = sqrt(pow(des_pose.pose.position.y - robot_pose.pose.pose.position.y, 2) + pow(des_pose.pose.position.x - robot_pose.pose.pose.position.x, 2));

    if (err_pos < dist_precision && err_yaw < yaw_precision)
    {
        done();
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
    check_goal();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle n;
    ros::Subscriber sub_laser = n.subscribe("/marrtino/scan", 1, laserReadCallback);
    ros::Subscriber sub_odom = n.subscribe("/marrtino/marrtino_base_controller/odom", 1, odomPoseCallback);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1);
    initMap();
    ros::Duration(1).sleep();
    ros::Rate rate(100);

    des_pose.pose.position.x = -1.327743;
    des_pose.pose.position.y = 3.2;
    des_pose.pose.position.z = 0;
    des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    des_pose.header.frame_id = "marrtino_map";
    des_pose.header.stamp = ros::Time::now();
    ROS_INFO("\t\t- Destination position = [%f, %f, %f]", des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z);
    ROS_INFO("\t\t- Destination orientation = [%f, %f, %f, %f]", des_pose.pose.orientation.x, des_pose.pose.orientation.y, des_pose.pose.orientation.z, des_pose.pose.orientation.w);
    tf::Pose current_goal;
    tf::poseMsgToTF(des_pose.pose, current_goal);

    ROS_INFO("\t\t- Destination yaw = %f", tf::getYaw(current_goal.getRotation()));

    while (ros::ok())
    {
        if (global_state != -1)
        {

            geometry_msgs::Twist msg;
            if (global_state == 0)
                msg = find_wall();
            else if (global_state == 1)
                msg = turn_right();
            else if (global_state == 2)
                msg = turn_left();
            else if (global_state == 3)
                msg = go_straight();
            else
                ROS_INFO("Unknown state!");

            cmd_pub.publish(msg);
            ros::spinOnce();

            rate.sleep();
        }
    }

    return 0;
}
