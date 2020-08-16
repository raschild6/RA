#include "ros/ros.h"
#include "homework1_test/msg_hm1.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/common_functions.h>


namespace apriltag_ros
{
    void APtag_arr(const AprilTagDetectionArray::ConstPtr& AP_arr);
}




/* ****************************** OLD ******************************
/* *****************************************************************

	void APtag_est_pos_cb(const geometry_msgs::PoseArray::ConstPtr& AP_est_pos){
		geometry_msgs::PoseArray APtag_est_pos = *AP_est_pos;
		int n_AP = 0;
		n_AP = AP_est_pos->poses.size();
		ROS_INFO(" ************* # AprilTag = %d ************* /n", n_AP);
		ROS_INFO("n=%d", n_AP);
		if(n_AP>0){  
			ROS_INFO("APtag est pos=[%f, %f, %f]/n",  AP_est_pos->poses[0].position.x,
													AP_est_pos->poses[0].position.y,
													AP_est_pos->poses[0].position.z);
		}
	}

/* *****************************************************************/