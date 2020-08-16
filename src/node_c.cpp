#include <iostream>
#include <algorithm>
#include <vector>
#include <fstream>
#include <ctime>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <stdio.h>


#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointXYZRGBA PointType;

int main(int argc, char **argv){	

	ros::init(argc, argv, "node_c");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("/kinect_cloud", 1);

	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_trasform;
	sensor_msgs::PointCloud2 cloud_world, cloud_output;
	sensor_msgs::PointCloud output, cloud_msgs;

	int choose = 10;
	choose = std::stoi(argv[1]);

	if(choose == 0){
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}else if(choose == 1){
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered2.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}else if(choose == 2){
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered3.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}else if(choose == 3){
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered4.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}else if(choose == 4){
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered5.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}else{
		if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/scene_not_filtered_2.pcd", *cloud) == -1){
			PCL_ERROR("Couldn't read scene file!");
			return 0;
		}else{
			ROS_INFO("Cloud loaded!");
		}
	}
	
	/*
	pcl::toROSMsg(*cloud, cloud_world);
	ROS_INFO("cloud_world size : %d", cloud_world.height * cloud_world.row_step);
	sensor_msgs::convertPointCloud2ToPointCloud(cloud_world, cloud_msgs);
	ROS_INFO("cloud_msgs size : %d", cloud_msgs.points.size());
	

	tf::TransformListener listener;
	try {
		listener.transformPointCloud("camera_link", cloud_msgs, output);
	} catch (tf::TransformException &ex) {
		ROS_INFO("Error Trasformation...%s",ex.what());
	}

	ROS_INFO("output size : %d", output.points.size());
	sensor_msgs::convertPointCloudToPointCloud2(output, cloud_output);
	ROS_INFO("cloud_output size : %d", cloud_output.height * cloud_output.row_step);
	cloud_output.header.frame_id = "camera_link";
	*/

	pcl::toROSMsg(*cloud, cloud_world);
	cloud_world.header.frame_id = "camera_rgb_optical_frame";

	ros::Rate loop_rate(4);
    while (n.ok()){

		pub.publish(cloud_world);
    	ros::spinOnce ();
    	loop_rate.sleep ();
    }

	return 0;

}

