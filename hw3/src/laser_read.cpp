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
unordered_map<string, float> regions;
int global_state = 0;
int move_to_wall = 1;

void initMap()
{
    regions["back"] = 0;
    regions["right"] = 0;
    regions["front"] = 0;
    regions["left"] = 0;
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
    ROS_INFO("back : %f", regions["back"]);
    ROS_INFO("right : %f", regions["right"]);
    ROS_INFO("front : %f", regions["front"]);
    ROS_INFO("left : %f", regions["left"]);
    string state_description = "";

    //float d = 0.17;  //good for min value
    float d = 0.3;
    /*
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

    if (regions["left"] < d && regions["right"] > d)
    {
        ROS_INFO("TAKE ACTION \t\t TURN RIGHT");

        move_to_wall = 0;
        state_description = "case 2 - left";
        change_state(1); //turn right
    }
    else if (regions["left"] > d && regions["right"] < d)
    {
        ROS_INFO("TAKE ACTION \t\t TURN LEFT");

        move_to_wall = 0;
        state_description = "case 3 - right";
        change_state(2); //turn left
    }
    else if (regions["left"] > d && regions["right"] > d)
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
    //ROS_INFO("LASER READ");
    range_min = msg.range_min;
    ROS_INFO("MIN RANGE: %f", range_min);
    range_max = msg.range_max;
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> back, right, front, left = {};
    //ROS_INFO("LASER RAYS: %d", msg.ranges.size());
    for (int i = 0; i < msg.ranges.size(); i++)
    {
        //ROS_INFO("msg at %d: %f", i, msg.ranges.at(i));
        if (msg.ranges.at(i) > range_min)
        {
            if (i < 50 || i >= 350)
                if ((i < 42 || i > 44) || (i < 357 || i > 359))
                    back.push_back(msg.ranges.at(i));
            if (i >= 50 && i < 150)
                if (i < 111 || i > 115)
                    right.push_back(msg.ranges.at(i));
            if (i >= 150 && i < 250)
                front.push_back(msg.ranges.at(i));
            if (i >= 250 && i < 350)
                if (i < 286 || i > 290)
                    left.push_back(msg.ranges.at(i));
        }
    }
    //ROS_INFO("BACK SIZE: %d", back.size());
    //ROS_INFO("MIN: %f", *std::min_element(std::begin(back), std::end(back)));
    regions["back"] = std::accumulate(std::begin(back), std::end(back), 0.0) / back.size();
    //ROS_INFO("RIGHT SIZE: %d", right.size());
    //ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["right"] = std::accumulate(std::begin(right), std::end(right), 0.0) / right.size();
    //ROS_INFO("FRONT SIZE: %d", front.size());
    //ROS_INFO("MIN: %f", *std::min_element(std::begin(front), std::end(front)));
    regions["front"] = std::accumulate(std::begin(front), std::end(front), 0.0) / front.size();
    //ROS_INFO("LEFT SIZE: %d", left.size());
    //ROS_INFO("MIN: %f", *std::min_element(std::begin(left), std::end(left)));
    regions["left"] = std::accumulate(std::begin(left), std::end(left), 0.0) / left.size();

    take_action();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle n;
    ros::Subscriber sub_laser = n.subscribe("/marrtino/scan", 1, laserReadCallback);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1);
    initMap();
    ros::Duration(1).sleep();
    ros::Rate rate(100);
    while (ros::ok())
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

    return 0;
}
