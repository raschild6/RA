#include <unordered_map> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <fstream>
#include <ctime>

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <apriltag_ros/common_functions.h>


using namespace std;

unordered_map<string,int> uIdMap;
vector<int> requestObjIds;
ofstream file;

ros::Publisher pub;
apriltag_ros::AprilTagDetectionArray tag_detection_array;


void publishResults(){

	pub.publish(tag_detection_array);
	//ros::spinOnce();
}


void InitFiles(){
	time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y_%H-%M-%S",timeinfo);
  	//file.open ("/home/michele/catkin_ws/src/metapackages/homework1_test/test/APtag_" + string(buffer) + ".txt", ios::app);
	
}

void APtag_arr(const apriltag_ros::AprilTagDetectionArray::ConstPtr& AP_arr){
	vector<apriltag_ros::AprilTagDetection> temp = AP_arr->detections;
	int count = 0;
	for (int i = 0; i < temp.size(); i++){
		vector<int> ids = temp.at(i).id;

		if(find(requestObjIds.begin(), requestObjIds.end(), ids[0]) != requestObjIds.end()){	
			geometry_msgs::PoseWithCovarianceStamped pose =  temp.at(i).pose;
			/*file << "id = " << ids[0] << " :" << endl;*/
			/*file << "\t\t- pose = [" << pose.pose.pose.position.x <<
								", " << pose.pose.pose.position.y <<
								", " <<	pose.pose.pose.position.z << "]" << endl;*/
			/*file << "\t\t- orient = [" << pose.pose.pose.orientation.x <<
								", " << pose.pose.pose.orientation.y <<
								", " <<	pose.pose.pose.orientation.z << 
								", " <<	pose.pose.pose.orientation.w << "]" << endl;*/

			ROS_INFO("id = %d :",ids[0]);
			ROS_INFO("\t\t- pose = [%f, %f, %f]", pose.pose.pose.position.x, 
													pose.pose.pose.position.y, 
													pose.pose.pose.position.z);
			ROS_INFO("\t\t- orient = [%f, %f, %f, %f]", pose.pose.pose.orientation.x,
														pose.pose.pose.orientation.y,
														pose.pose.pose.orientation.z,
														pose.pose.pose.orientation.w);
			count++;

			apriltag_ros::AprilTagDetection tag_detection;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "camera_link";
			tag_detection.pose = pose;
			tag_detection.id = ids;
			tag_detection_array.detections.push_back(tag_detection);
		}else{
			///*file << "id = " << ids[0] << " Obj found but not searched!" << endl;*/
			ROS_INFO("id = %d Obj found but not searched!", ids[0]);
		}

		publishResults();
	}
	if(count < requestObjIds.size()){
		/*file << "Number of objects not found : " << (requestObjIds.size() - count) << endl;*/
		ROS_INFO("Number of objects not found : %d", (requestObjIds.size() - count));
	}
	/*file << "*******************************************************************" << endl;*/
	ROS_INFO("*******************************************************************");
}

void IdsFromFrameIds(int argc, char **argv){
	for (int i = 1; i < argc; i++){
			if (uIdMap.find(argv[i]) != uIdMap.end()) 
        		requestObjIds.push_back(uIdMap.at(argv[i])); 
		    else
        		ROS_INFO("'%s' probably error name! Obj not considered!\n", argv[i]);
	}
}

void InitializeMap(){
	uIdMap["red_cube_0"] 	= 0;
	uIdMap["red_cube_1"] 	= 1;
	uIdMap["red_cube_2"] 	= 2;
	uIdMap["red_cube_3"] 	= 3;
	uIdMap["yellow_cyl_0"] 	= 4;
	uIdMap["yellow_cyl_1"] 	= 5;
	uIdMap["green_prism_0"] = 6;
	uIdMap["green_prism_1"] = 7;
	uIdMap["green_prism_2"] = 8;
	uIdMap["blue_cube_0"] 	= 9;
	uIdMap["blue_cube_1"] 	= 10;
	uIdMap["blue_cube_2"] 	= 11;
	uIdMap["blue_cube_3"] 	= 12;
	uIdMap["red_prism_0"] 	= 13;
	uIdMap["red_prism_1"] 	= 14;
	uIdMap["red_prism_2"] 	= 15;
}

int main(int argc, char **argv){	
	
	ROS_INFO("");
	ROS_INFO("Obj requested :");
	for (int i = 1; i < argc; i++){
			ROS_INFO(" - %s", argv[i]);
	}
	ROS_INFO("");

	if(argc > 1){

		InitializeMap();
		IdsFromFrameIds(argc, argv);

		for (int i = 0; i < requestObjIds.size(); i++){
			ROS_INFO("id request: %d", requestObjIds[i]);
		}

		ROS_INFO("");
		
		//InitFiles();		// For debugging save all output in a /*file

		ros::init(argc, argv, "node_b");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1000, APtag_arr);
		pub = pub = n.advertise<apriltag_ros::AprilTagDetectionArray> ("/pose_objects", 1);
		ros::spin();

	}else
		ROS_INFO("There are no objects required!");
	
	//file.close();
	return 0;

}

