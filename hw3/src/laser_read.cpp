#include "follow_wall.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <string>
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
void initMap()
{
    regions["right"] = 0;
    regions["fright"] = 0;
    regions["front"] = 0;
    regions["fleft"] = 0;
    regions["left"] = 0;
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
void take_action()
{
    ROS_INFO("TAKE ACTION");
    ROS_INFO("right : %f", regions["right"]);
    ROS_INFO("fright : %f", regions["fright"]);
    ROS_INFO("front : %f", regions["front"]);
    ROS_INFO("left : %f", regions["left"]);
    ROS_INFO("fleft : %f", regions["left"]);
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
    ROS_INFO("LASER READ");
    range_min = msg.range_min;
    range_max = msg.range_max;
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> right, fright, front, fleft, left = {};
    ROS_INFO("LASER RAYS: %d", msg.ranges.size());
    for (int i = 0; i < msg.ranges.size(); i++)
    {
        ROS_INFO("msg at %d: %f", i, msg.ranges.at(i));
        if (msg.ranges.at(i) > range_min)
        {
            if (i < msg.ranges.size()/5)
                left.push_back(msg.ranges.at(i));
            if (i >= msg.ranges.size()/5 && i < 2*(msg.ranges.size()/5))
                fleft.push_back(msg.ranges.at(i));
            if (i >= 2*(msg.ranges.size()/5) && i < 3*(msg.ranges.size()/5))
                front.push_back(msg.ranges.at(i));
            if (i >= 3*(msg.ranges.size()/5) && i < 4*(msg.ranges.size()/5))
                fright.push_back(msg.ranges.at(i));
            if (i >= 4*(msg.ranges.size()/5) && i < 5*(msg.ranges.size()/5))
                right.push_back(msg.ranges.at(i));
        }
    }
    ROS_INFO("RIGHT SIZE: %d", right.size());
    ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["right"] = *std::min_element(std::begin(right), std::end(right));
    ROS_INFO("FRIGHT SIZE: %d", fright.size());
    ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["fright"] = *std::min_element(std::begin(fright), std::end(fright));
    ROS_INFO("FRONT SIZE: %d", front.size());
    ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["front"] = *std::min_element(std::begin(front), std::end(front));
    ROS_INFO("FLEFT SIZE: %d", fleft.size());
    ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["fleft"] = *std::min_element(std::begin(fleft), std::end(fleft));
    ROS_INFO("LEFT SIZE: %d", left.size());
    ROS_INFO("MIN: %f", *std::min_element(std::begin(right), std::end(right)));
    regions["left"] = *std::min_element(std::begin(left), std::end(left));

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

    return 0;
}
