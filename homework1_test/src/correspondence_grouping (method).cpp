#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>



typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

//Algorithm params
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);



double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &scene)
{
	//
	//  Set up resolution invariance
	//
	float resolution = static_cast<float> (computeCloudResolution (model));
	if (resolution != 0.0f){
		model_ss_   *= resolution;
		scene_ss_   *= resolution;
		rf_rad_     *= resolution;
		descr_rad_  *= resolution;
		cg_size_    *= resolution;
	}

	ROS_INFO ( "Model resolution:       %f", resolution);
	ROS_INFO ( "Model sampling size:    %f", model_ss_ );
	ROS_INFO ( "Scene sampling size:    %f", scene_ss_ );
	ROS_INFO ( "LRF support radius:     %f", rf_rad_ );
	ROS_INFO ( "SHOT descriptor radius: %f", descr_rad_ );
	ROS_INFO ( "Clustering bin size:    %f", cg_size_);

   	//pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
	
	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.filter (*model_keypoints);
	ROS_INFO ( "Model total points: %d ; Selected Keypoints: %d", model->size (), model_keypoints->size ());

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	ROS_INFO ( "Scene total points: %d ; Selected Keypoints: %d", scene->size (), scene_keypoints->size ());


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at (i).descriptor[0])){ //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	ROS_INFO ( "Correspondences found: %d", model_scene_corrs->size ());

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_){
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize (cg_size_);
		clusterer.setHoughThreshold (cg_thresh_);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (model_keypoints);
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	}
	else{ // Using GeometricConsistency
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	ROS_INFO ("Model instances found: %d", rototranslations.size ());
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		ROS_INFO ( "\n    Instance %d :", i + 1);
		ROS_INFO ( "        Correspondences belonging to this instance: %d", clustered_corrs[i].size ());

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		ROS_INFO ("");
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		ROS_INFO ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		ROS_INFO ("");
		ROS_INFO ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}

	//
	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.addPointCloud (scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	if (show_correspondences_ || show_keypoints_){
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_){
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}

	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_){
			for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j){
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
			PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}
}






























#include <sstream>

#include "ros/ros.h"
#include "homework1_test/msg_hm1.h"
#include <homework1_test/passthrough_test.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <thread> 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

//Algorithm params
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (false);
float model_ss_ (0.18f);
float scene_ss_ (0.13f);
float rf_rad_ (0.06f);
float descr_rad_ (2.6f);
float cg_size_ (3.01f);
float cg_thresh_ (5.0f);


pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());

pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
std::vector<pcl::Correspondences> clustered_corrs;



ros::Publisher pub;
pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
bool once = false;
int nView = 3;

void readMeshes(std::string file_name){

	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

  	if (pcl::io::loadPCDFile<PointType> (file_name, *cloud) == -1) {
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  	}

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (cloud);
   	while (!viewer.wasStopped ()){
   	}

}

double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud){
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &scene){
	//pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());

	if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd", *model) == -1){
    	PCL_ERROR("Couldn't read .pcd file model");
  	}

	//
	//  Set up resolution invariance
	//
	float resolution = static_cast<float> (computeCloudResolution (model));
	if (resolution != 0.0f){
		model_ss_   *= resolution;
		scene_ss_   *= resolution;
		rf_rad_     *= resolution;
		descr_rad_  *= resolution;
		cg_size_    *= resolution;
	}

	ROS_INFO ( "Model resolution:       %f", resolution);
	ROS_INFO ( "Model sampling size:    %f", model_ss_ );
	ROS_INFO ( "Scene sampling size:    %f", scene_ss_ );
	ROS_INFO ( "LRF support radius:     %f", rf_rad_ );
	ROS_INFO ( "SHOT descriptor radius: %f", descr_rad_ );
	ROS_INFO ( "Clustering bin size:    %f", cg_size_);
	
	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.filter (*model_keypoints);
	ROS_INFO ( "Model total points: %d ; Selected Keypoints: %d", model->size (), model_keypoints->size ());

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	ROS_INFO ( "Scene total points: %d ; Selected Keypoints: %d", scene->size (), scene_keypoints->size ());


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at (i).descriptor[0])){ //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	ROS_INFO ( "Correspondences found: %d", model_scene_corrs->size ());

	//
	//  Actual Clustering
	//
	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	//std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_){
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize (cg_size_);
		clusterer.setHoughThreshold (cg_thresh_);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (model_keypoints);
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	}
	else{ // Using GeometricConsistency
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	ROS_INFO ("Model instances found: %d", rototranslations.size ());
	/**/
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		ROS_INFO ( "\n    Instance %d :", i + 1);
		ROS_INFO ( "        Correspondences belonging to this instance: %d", clustered_corrs[i].size ());

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		ROS_INFO ("");
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		ROS_INFO ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		ROS_INFO ("");
		ROS_INFO ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}

	//
	//  Visualization
	//**/		
	viewer.removeAllShapes();
	viewer.removeAllPointClouds();
	//pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.addPointCloud (cloud_filtered, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	if (show_correspondences_ || show_keypoints_){
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_){
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");

	}


	pcl::PointCloud<PointType>::Ptr rotated_model_permanent (new pcl::PointCloud<PointType> ());
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		pcl::transformPointCloud (*model, *rotated_model_permanent, rototranslations[0]);
		
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_){
			for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j){
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
			PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}

	ros::NodeHandle nh;

	//create ros messages.
	sensor_msgs::PointCloud2 sceneCloud, modelCloud, offSceneModelCloud;

	//change point cloud, make it into a point cloud 2 message.
	pcl::toROSMsg(*cloud_filtered, sceneCloud);
	pcl::toROSMsg(*rotated_model_permanent, modelCloud);
	pcl::toROSMsg(*off_scene_model, offSceneModelCloud);
	//set frame id of published message
	sceneCloud.header.frame_id = "map";
	modelCloud.header.frame_id = "map";
	offSceneModelCloud.header.frame_id = "map";

	//set up pubs.
	ros::Publisher scenePub = nh.advertise<sensor_msgs::PointCloud2>("scene", 1);
	ros::Publisher modelPub = nh.advertise<sensor_msgs::PointCloud2>("model", 1);
	ros::Publisher offSceneModelPub = nh.advertise<sensor_msgs::PointCloud2>("offSceneModel", 1);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		//update time
		sceneCloud.header.stamp = ros::Time::now();
		modelCloud.header.stamp = ros::Time::now();
		offSceneModelCloud.header.stamp = ros::Time::now();

		scenePub.publish(sceneCloud);
		modelPub.publish(modelCloud);
		offSceneModelPub.publish(offSceneModelCloud);
		ros::spinOnce();
	}
}

void Plan_segmentation(const pcl::PointCloud<PointType>::ConstPtr &cloud){
	/*
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	}

	ROS_INFO("Model coefficients: [%f,%f,%f,%f]", coefficients->values[0], 
												  coefficients->values[1],
												  coefficients->values[2], 
												  coefficients->values[3]);

	ROS_INFO("Model inliers: %d", inliers->indices.size ());
	for (std::size_t i = 0; i < inliers->indices.size (); ++i)
		ROS_INFO("%d    - point [%f,%f,%f]",inliers->indices[i], cloud->points[inliers->indices[i]].x,
																 cloud->points[inliers->indices[i]].y,
																 cloud->points[inliers->indices[i]].z);
	*/
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){

	pcl::PCLPointCloud2 topic_cloud;
	pcl_conversions::toPCL(*MScloud, topic_cloud);
    pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(topic_cloud, *temp_cloud);

	/*
	ROS_INFO("Cloud BEFORE filtering: ");
  	for (std::size_t i = 0; i < temp_cloud->points.size (); ++i)
    ROS_INFO("\t\t- points = [%f, %f, %f]", temp_cloud->points[i].x, 
                        					temp_cloud->points[i].y, 
                        					temp_cloud->points[i].z);/*	
	
	//Passthrough cloud_toFilter;
	//cloud_toFilter.cloud -> points = temp_cloud -> points;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_toFilter.filtering();
	*/

	//pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
	
	/* ************************************** Filter on axes Z *************************
	// Create the filtering object
	pcl::PassThrough<PointType> pass;
	pass.setInputCloud (temp_cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.98);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);
	*/ 
	
	// *************************************** Filter on box X,Y,Z *************************
	// Define min and max for X, Y and Z
	float minX = -0.6, minY = -0.5, minZ = 0.0;
	float maxX = +0.6, maxY = +0.3, maxZ = +1.98;

	pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	boxFilter.setInputCloud(temp_cloud);
	boxFilter.filter(*cloud_filtered);


	/* *************************************** Downsampling *************************
	pcl::PCLPointCloud2Ptr temp_pcl (new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2Ptr temp_pcl_filtered (new pcl::PCLPointCloud2 ());

	pcl::toPCLPointCloud2(*cloud_filtered, *temp_pcl);

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (temp_pcl);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*temp_pcl_filtered);

	pcl::PointCloud<PointType>::Ptr temp_filtered_downsamp(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(*temp_pcl_filtered, *temp_filtered_downsamp);

	// Convert to ROS data type
  	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish(output);

	/** /
	//readMeshes("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon.pcd");
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (cloud_filtered);
   	while (!viewer.wasStopped ()){
   	}
	/**/
	if(nView == 0){
		once = false;
	}
	if(once){
		//once = false;
		//for(float i = 0.09; i < 0.1; ){
			model_ss_ = (float)0.18;
			scene_ss_ = (float)0.13;
			rf_rad_ = (float)(1.06);
			descr_rad_ = (float)(3.57);
			cg_size_ = (float)3.01;
			cg_thresh_ = (float)5.0;
			nView = nView - 1;
		//	ROS_INFO ( "" );			
		//	ROS_INFO ( "***************** CYCLE NUMBER ************ : %f", descr_rad_ );

			Correspondence_Grouping(cloud_filtered);
		//	i = i + 0.01;
		//}
	}
	/*
	ROS_INFO("Cloud AFTER filtering: ");
  	for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
    ROS_INFO("\t\t- points_filtered = [%f, %f, %f]", cloud_filtered->points[i].x, 
                        							 cloud_filtered->points[i].y, 
                        							 cloud_filtered->points[i].z);
	*/
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Pcl_camera);
	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	ros::spin();

	return 0;

}
























#include <sstream>

#include "ros/ros.h"
#include "homework1_test/msg_hm1.h"
#include <homework1_test/passthrough_test.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <thread> 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

//Algorithm params
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (false);
float model_ss_ (0.18f);
float scene_ss_ (0.13f);
float rf_rad_ (0.06f);
float descr_rad_ (2.6f);
float cg_size_ (3.01f);
float cg_thresh_ (5.0f);


//pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
bool update = false;

ros::Publisher pub;
bool once = true;
int nView = -1;

void readMeshes(std::string file_name){

	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

  	if (pcl::io::loadPCDFile<PointType> (file_name, *cloud) == -1) {
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  	}

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (cloud);
   	while (!viewer.wasStopped ()){
   	}

}

void RealTimeViewer(){

}

double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud){
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &scene){
	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());

	if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_10k.pcd", *model) == -1){
    	PCL_ERROR("Couldn't read .pcd file model");
  	}

	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	//
	//  Set up resolution invariance
	//
	float resolution = static_cast<float> (computeCloudResolution (model));
	if (resolution != 0.0f){
		model_ss_   *= resolution;
		scene_ss_   *= resolution;
		rf_rad_     *= resolution;
		descr_rad_  *= resolution;
		cg_size_    *= resolution;
	}

	ROS_INFO ( "Model resolution:       %f", resolution);
	ROS_INFO ( "Model sampling size:    %f", model_ss_ );
	ROS_INFO ( "Scene sampling size:    %f", scene_ss_ );
	ROS_INFO ( "LRF support radius:     %f", rf_rad_ );
	ROS_INFO ( "SHOT descriptor radius: %f", descr_rad_ );
	ROS_INFO ( "Clustering bin size:    %f", cg_size_);
	
	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.filter (*model_keypoints);
	ROS_INFO ( "Model total points: %d ; Selected Keypoints: %d", model->size (), model_keypoints->size ());

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	ROS_INFO ( "Scene total points: %d ; Selected Keypoints: %d", scene->size (), scene_keypoints->size ());


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at (i).descriptor[0])){ //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.15f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	ROS_INFO ( "Correspondences found: %d", model_scene_corrs->size ());

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_){
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize (cg_size_);
		clusterer.setHoughThreshold (cg_thresh_);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (model_keypoints);
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	}
	else{ // Using GeometricConsistency
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	ROS_INFO ("Model instances found: %d", rototranslations.size ());
	/**/
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		ROS_INFO ( "\n    Instance %d :", i + 1);
		ROS_INFO ( "        Correspondences belonging to this instance: %d", clustered_corrs[i].size ());

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		ROS_INFO ("");
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		ROS_INFO ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		ROS_INFO ("");
		ROS_INFO ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}

	//
	//  Visualization
	//**/
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.removeAllShapes();
	viewer.removeAllPointClouds();

	viewer.addPointCloud (scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	if (show_correspondences_ || show_keypoints_){
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_){
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");

	}


	pcl::PointCloud<PointType>::Ptr rotated_model_permanent (new pcl::PointCloud<PointType> ());
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		pcl::transformPointCloud (*model, *rotated_model_permanent, rototranslations[0]);
		
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_){
			for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j){
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
			PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}
	while(!viewer.wasStopped()){
		viewer.spinOnce ();
	}
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){

	pcl::PCLPointCloud2 topic_cloud;
	pcl_conversions::toPCL(*MScloud, topic_cloud);
    pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(topic_cloud, *temp_cloud);

	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
	
	// *************************************** Filter on box X,Y,Z *************************
	// Define min and max for X, Y and Z
	float minX = -0.6, minY = -0.5, minZ = 0.0;
	float maxX = +0.6, maxY = +0.3, maxZ = +1.98;

	pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	boxFilter.setInputCloud(temp_cloud);
	boxFilter.filter(*cloud_filtered);

	// Convert to ROS data type
  	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish(output);

	if(nView == 0){
		once = false;
	}
	if(once){

		model_ss_ = (float)0.18;
		scene_ss_ = (float)0.13;
		rf_rad_ = (float)0.06;
		descr_rad_ = (float)2.06;
		cg_size_ = (float)3.01;
		cg_thresh_ = (float)5.0;
		nView = nView - 1;

		Correspondence_Grouping(cloud_filtered);

	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Pcl_camera);
	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	ros::spin();

	return 0;

}















#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <iostream>
#include "omp.h"


typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;


//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (true);
bool use_hough_ (true);
float model_ss_ ( 0.005);
float scene_ss_ ( 0.005);
float rf_rad_ (0.04);
float descr_rad_ (0.04);
float cg_size_ (0.02);
float cg_thresh_ (5.0f);
bool narf(false);
bool sift(false);
bool fpfh(true);
const int distance = 2000; //kinect cut-off distance



double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud); 

  #pragma omp parallel for
  for (size_t i = 0; i < cloud->size (); ++i){
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    { 
      #pragma omp critical
      {
      res += sqrt (sqr_distances[1]);
      ++n_points;
      }
    }
  }

  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


void visualize_keypoints(pcl::PointCloud<PointType>::Ptr model,
                          pcl::PointCloud<PointType>::Ptr model_keypoints,
                          pcl::PointCloud<PointType>::Ptr scene,
                          pcl::PointCloud<PointType>::Ptr scene_keypoints) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> model_viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));


  model_viewer->setBackgroundColor(255, 255, 255);
  scene_viewer->setBackgroundColor(255, 255, 255);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> model_rgb(model);
  model_viewer->addPointCloud<pcl::PointXYZRGB>(model, model_rgb, "model cloud");
  model_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model cloud");

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> scene_rgb(scene);
  scene_viewer->addPointCloud<pcl::PointXYZRGB>(scene, scene_rgb, "scene cloud");
  scene_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene cloud");


  // Visualization of keypoints along with the original cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> model_keypoints_color_handler (model_keypoints, 0, 255, 0);
  model_viewer->addPointCloud(model_keypoints, model_keypoints_color_handler, "model_keypoints");
  model_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "model_keypoints");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_keypoints_color_handler (scene_keypoints, 0, 255, 0);
  scene_viewer->addPointCloud(scene_keypoints, scene_keypoints_color_handler, "model_keypoints");
  scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "model_keypoints");


   while (!scene_viewer->wasStopped() )
  {
    model_viewer->spinOnce();
    scene_viewer->spinOnce();
  }

}


template <class T, class Estimator>
class KeyDes
{
public:
  typedef pcl::PointCloud<T> PT;
  typedef pcl::PointCloud<PointType> P;
  typedef pcl::PointCloud<NormalType> PN;
  typename PT::Ptr model_descriptors;
  typename PT::Ptr scene_descriptors;
  typename P::Ptr model ;
  typename P::Ptr model_keypoints;
  typename P::Ptr scene;
  typename P::Ptr scene_keypoints;
  typename PN::Ptr model_normals;
  typename PN::Ptr scene_normals;


  KeyDes(P::Ptr model, P::Ptr model_keypoints, P::Ptr scene, P::Ptr scene_keypoints, PN::Ptr model_normals, PN::Ptr scene_normals ):
            model_descriptors (new PT ()),
            scene_descriptors (new PT ()),
            model(model),
            model_keypoints(model_keypoints),
            scene(scene),
            scene_keypoints(scene_keypoints),
            model_normals(model_normals),
            scene_normals(scene_normals){}

  pcl::CorrespondencesPtr run()
  {

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    //create scene descriptors
    std::cout << "calculating fpfh scene descriptors "  <<std::endl;
    Estimator est;
    est.setInputCloud(scene_keypoints);
    est.setSearchSurface(scene);
    est.setInputNormals(scene_normals);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    est.setSearchMethod(tree);
    est.setRadiusSearch (0.05);
    est.compute (*scene_descriptors);


    //create model descriptors
    std::cout << "calculating fpfh model descriptors "  <<std::endl;
    est.setInputCloud(model_keypoints);
    est.setSearchSurface(model);
    est.setInputNormals(model_normals);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
    est.setSearchMethod(tree2);
    est.compute (*model_descriptors);

    pcl::KdTreeFLANN<T> match_search;
    std::cout <<"calculated " << model_descriptors->size() << " for the model and " << scene_descriptors->size() << " for the scene" <<std::endl;

    //  Find Model-Scene Correspondences with KdTree
    std::cout << "calculating correspondences "  <<std::endl;


    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      
      int found_neighs = match_search.nearestKSearch (scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
    return model_scene_corrs;

  }

  
};


int main( int argc, char** argv )
{

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  

  pcl::VoxelGrid<pcl::PointXYZRGB> down_sampler;
  int save = 1;//atoi(argv[3]);


  //load and filter model and scene
  if (pcl::io::loadPCDFile (argv[1], *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  } 
   if (pcl::io::loadPCDFile (argv[2], *scene) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  } 
   
  std::cout << "cloud model dimensions " << model->height<< " x " << model->width << " number of points " << model->points.size() <<std::endl;

  if (float resolution = static_cast<float> (computeCloudResolution (model)) != 0.0f) { //see c++11 standard
    model_ss_   *= resolution;
    scene_ss_   *= resolution;
    rf_rad_     *= resolution;
    descr_rad_  *= resolution;
    cg_size_    *= resolution;
  
    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }


  //  Compute Normals
  std::cout << "calculating model normals... "  <<std::endl;
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);
  if(save == 0)
    pcl::io::savePCDFileASCII ("model_sampled_normals.pcd", *model_normals);

  std::cout << "calculating scene normals... "  <<std::endl;
  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);
  if(save == 0)
    pcl::io::savePCDFileASCII ("scene_sampled_normals.pcd", *scene_normals);


  //pcl::removeNaNNormalsFromPointCloud(*model_normals,*model_normals, indexes2);
  //pcl::removeNaNNormalsFromPointCloud(*scene_normals,*scene_normals, indexes2);


  if(narf){

    //NARF
    std::cout << "finding narf keypoints..."<< std::endl;

    pcl::PointCloud<int> scene_keypoint_indices;
    pcl::PointCloud<int> model_keypoint_indices;

    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (scene->sensor_origin_[0],
                                                                scene->sensor_origin_[1],
                                                                scene->sensor_origin_[2])) *
                                                                Eigen::Affine3f (scene->sensor_orientation_);
    Eigen::Affine3f model_sensor_pose (Eigen::Affine3f::Identity ());


    model_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (model->sensor_origin_[0],
                                                                model->sensor_origin_[1],
                                                                model->sensor_origin_[2])) *
                                                                Eigen::Affine3f (model->sensor_orientation_);
    float support_size = 0.2f;
    bool rotation_invariant = true;
    boost::shared_ptr<pcl::RangeImage> scene_range_image_ptr (new pcl::RangeImage);
    boost::shared_ptr<pcl::RangeImage> model_range_image_ptr (new pcl::RangeImage);

    pcl::RangeImage& scene_range_image = *scene_range_image_ptr;
    pcl::RangeImage& model_range_image = *model_range_image_ptr;

    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;

    
    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&scene_range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;

    scene_range_image.createFromPointCloud(*scene, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                       scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
    model_range_image.createFromPointCloud(*model, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                       model_sensor_pose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
    scene_range_image.setUnseenToMaxRange();
    model_range_image.setUnseenToMaxRange();


    narf_keypoint_detector.setRangeImage (&scene_range_image);
    narf_keypoint_detector.compute (scene_keypoint_indices);

    narf_keypoint_detector.setRangeImage (&model_range_image);
    narf_keypoint_detector.compute (model_keypoint_indices);
    
    
    scene_keypoints->points.resize (scene_keypoint_indices.points.size ());
    model_keypoints->points.resize (model_keypoint_indices.points.size ());

    #pragma omp parallel for
    for (size_t i=0; i<scene_keypoint_indices.points.size (); ++i)
      scene_keypoints->points[i].getVector3fMap () = scene_range_image.points[scene_keypoint_indices.points[i]].getVector3fMap ();
    #pragma omp parallel for
    for (size_t i=0; i<model_keypoint_indices.points.size (); ++i)
      model_keypoints->points[i].getVector3fMap () = model_range_image.points[model_keypoint_indices.points[i]].getVector3fMap ();
      
  }else if (sift){

    //SIFT
    std::cout << "finding sift keypoints..."<< std::endl;


    // Parameters for sift computation
    const float min_scale = 10;
    const int n_octaves = 6;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.005f;

    pcl::PointCloud<pcl::PointWithScale> model_result;
    pcl::PointCloud<pcl::PointWithScale> scene_result;



    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(model);
    sift.compute(model_result);

    sift.setInputCloud(scene);
    sift.compute(scene_result);

    copyPointCloud(model_result, *model_keypoints);
    copyPointCloud(scene_result, *scene_keypoints);


  }else{

    //  Downsample Clouds to Extract keypoints

    std::cout << "finding uniform sampled keypoints..."<< std::endl;

    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<PointType> uniform_sampling;

    //model
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);

    //scene
    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);

}

  std::cout << "\tfound " << scene_keypoints->points.size() << " keypoints in the scene and " << model_keypoints->points.size() << " in the model"<<std::endl;
  visualize_keypoints(model, model_keypoints, scene, scene_keypoints);


  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  if(fpfh) {
    KeyDes<pcl::FPFHSignature33, pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> > est(model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
    model_scene_corrs = est.run();
  }else{
    KeyDes<pcl::SHOT352, pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> > est(model, model_keypoints, scene, scene_keypoints, model_normals, scene_normals);
    model_scene_corrs = est.run();
  }


  //  Actual Clustering
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  std::cout << "Starting to cluster..." <<std::endl;
  //  Using Hough3D
  if (use_hough_)
  {
    //  Compute (Keypoints) Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);


    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    std::cout << "computed hough BOARD on model" <<std::endl;

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    std::cout << "computed hough BOARD on scene" <<std::endl;

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    std::cout << "prepared Hough for clustering" <<std::endl;

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);

    std::cout << "clustered" <<std::endl;

  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }




  //  Output results
    std::cout << "Model instances found: " << clustered_corrs.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

    //  Visualization
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1000,0,-1000), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1000,0,-1000), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
    
    if (show_correspondences_)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  
  return 0;
}























#include <sstream>
#include <iostream>
#include <thread> 
#include <fstream>
#include <stdlib.h>

#include "ros/ros.h"
#include "homework1_test/msg_hm1.h"
#include <homework1_test/passthrough_test.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>	
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

//Algorithm params
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (false);

float model_ss_ (1.01f);
float scene_ss_ (1.01f);
float rf_rad_ (3.51f);
float descr_rad_ (5.01f);
float cg_size_ (5.01f);
float cg_thresh_ (4.01f);

int argc_g; 
std::string argv_g[6];

int count = 1;
std::ofstream file,file2;
ros::Publisher pub;

int best2 = 100;


void readMeshes(std::string file_name){

	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);

  	if (pcl::io::loadPCDFile<PointType> (file_name, *cloud) == -1) {
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  	}

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (cloud);
   	while (!viewer.wasStopped ()){
   	}

}

double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud){
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &scene){
	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());

	if (pcl::io::loadPCDFile<PointType> ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd", *model) == -1){
    	PCL_ERROR("Couldn't read .pcd file model");
  	}

	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	//
	//  Set up resolution invariance
	//
	float resolution = static_cast<float> (computeCloudResolution (model));
	if (resolution != 0.0f){
		model_ss_   *= resolution;
		scene_ss_   *= resolution;
		rf_rad_     *= resolution;
		descr_rad_  *= resolution;
		cg_size_    *= resolution;
	}

	ROS_INFO ( "Model resolution:       %f", resolution);
	ROS_INFO ( "Model sampling size:    %f", model_ss_ );
	ROS_INFO ( "Scene sampling size:    %f", scene_ss_ );
	ROS_INFO ( "LRF support radius:     %f", rf_rad_ );
	ROS_INFO ( "SHOT descriptor radius: %f", descr_rad_ );
	ROS_INFO ( "Clustering bin size:    %f", cg_size_);
	ROS_INFO ( "Clustering bin size:    %f", cg_thresh_);
	
	//
	//  Compute Normals
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.filter (*model_keypoints);
	ROS_INFO ( "Model total points: %d ; Selected Keypoints: %d", model->size (), model_keypoints->size ());

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	ROS_INFO ( "Scene total points: %d ; Selected Keypoints: %d", scene->size (), scene_keypoints->size ());


	//
	//  Compute Descriptor for keypoints
	//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at (i).descriptor[0])){ //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.15f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	
	ROS_INFO ( "Correspondences found: %d", model_scene_corrs->size ());

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_){
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize (cg_size_);
		clusterer.setHoughThreshold (cg_thresh_);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (model_keypoints);
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	}
	else{ // Using GeometricConsistency
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	ROS_INFO ("Model instances found: %d", rototranslations.size ());
	/**/
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		ROS_INFO ( "\n    Instance %d :", i + 1);
		ROS_INFO ( "        Correspondences belonging to this instance: %d", clustered_corrs[i].size ());

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		ROS_INFO ("");
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		ROS_INFO ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		ROS_INFO ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		ROS_INFO ("");
		ROS_INFO ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}


	/**/
	file << "          Correspondences found:     " 		<< model_scene_corrs->size () 	<< endl;
	file <<	"          Model instances found:     "		 	<< rototranslations.size () 	<< endl;
	file << endl;
	
	if(rototranslations.size() != 0 ){
		//best2 = (int)rototranslations.size ();
		file2 << "TEST " << count <<"   PARAMS :"	<< endl;
		file2 << "     resolution    "	<< resolution	<< endl;
		file2 << "     model_ss_     " 	<<	model_ss_ 	<< endl;
		file2 << "     scene_ss_     "	<<	scene_ss_ 	<< endl;
		file2 << "     rf_rad_       " 	<< 	rf_rad_  	<< endl;
		file2 << "     descr_rad_    " 	<< 	descr_rad_	<< endl;
		file2 << "     cg_size_      " 	<< 	cg_size_	<< endl;
		file2 << "     cg_thresh_    " 	<< cg_thresh_ 	<< endl;

		file2 << "          Correspondences found:     " 		<< model_scene_corrs->size () 	<< endl;
		file2 << "          Model instances found:     " 		<< rototranslations.size () 	<< endl;
		file2 << endl;
	}
	/**/


	//
	//  Visualization
	/**/
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");	

	viewer.removeAllShapes();
	viewer.removeAllPointClouds();

	viewer.addPointCloud (scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	if (show_correspondences_ || show_keypoints_){
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_){
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");

	}


	pcl::PointCloud<PointType>::Ptr rotated_model_permanent (new pcl::PointCloud<PointType> ());
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		pcl::transformPointCloud (*model, *rotated_model_permanent, rototranslations[0]);
		
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_){
			for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j){
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
			PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	/**/
}

pcl::PointCloud<PointType>::Ptr Plan_segmentation(const pcl::PointCloud<PointType>::ConstPtr &cloud){

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_outliers (new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	

	// Create the segmentation object
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0){
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	}

	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);			
	extract.filter (*cloud_temp);	


 	// Estimate point normals
	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  	ne.setSearchMethod (tree);
  	ne.setInputCloud (cloud_temp);
  	ne.setKSearch (50);
  	ne.compute (*cloud_normals);

	pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg2; 
	seg2.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg2.setMaxIterations(100);
	seg2.setMethodType (pcl::SAC_RANSAC);
	seg2.setDistanceThreshold (0.01);
	seg2.setInputCloud (cloud_temp);
	seg2.setInputNormals (cloud_normals);
	seg2.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0){
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	}

	pcl::ExtractIndices<PointType> extract2;
	extract2.setInputCloud (cloud_temp);
	extract2.setIndices (inliers);
	extract2.setNegative (true);				// Extract the outliers
	extract2.filter (*cloud_outliers);			// cloud_outliers contains everything but the plane

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	extract_normals.setNegative (true);
  	extract_normals.setInputCloud (cloud_normals);
  	extract_normals.setIndices (inliers);
  	extract_normals.filter (*cloud_normals2);

	return cloud_outliers;
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){
	
	
	pcl::PCLPointCloud2 topic_cloud;
	pcl_conversions::toPCL(*MScloud, topic_cloud);
    pcl::PointCloud<PointType>::Ptr save_cloud(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(topic_cloud, *save_cloud);

	pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_not_filtered.pcd", *save_cloud);
	
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	sensor_msgs::PointCloud2 cloud_world, output;
	
    ros::Rate rate(30.0);
	ros::Duration timeout(1.0);
	try {
		transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time(0), timeout);
		transformStamped.header.frame_id = "world";
		tf2::doTransform(*MScloud, cloud_world, transformStamped);

	} catch (tf2::TransformException &ex) {
	}

    pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment2 (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);

	pcl::PCLPointCloud2 cloud_pcl_world;
	pcl_conversions::toPCL(cloud_world, cloud_pcl_world);
    pcl::fromPCLPointCloud2(cloud_pcl_world, *temp_cloud);

	// *************************************** Filter on box X,Y,Z *************************
	// Define min and max for X, Y and Z 
	/*
	float minX = -0.6, minY = -0.5, minZ = 0.0;
	float maxX = +0.6, maxY = +0.3, maxZ = +1.98;

	pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	boxFilter.setInputCloud(cloud_segment);
	boxFilter.filter(*cloud_filtered);
	/**/

	// Create the filtering object in X
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;
	pass.setInputCloud (temp_cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.6, + 0.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_segment);
	/**/
	// Create the filtering object in Y
	pass2.setInputCloud (cloud_segment);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (-1.2, +1.2);				//CONTROLLA BENE QUESTA MISURA
	//pass.setFilterLimitsNegative (true);
	pass2.filter (*cloud_segment2);
	/**/

	cloud_filtered = Plan_segmentation(cloud_segment2);

	pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_filtered.pcd", *cloud_filtered);

	// Convert to ROS data type
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish(output);

	/*
	file2.open ("/home/michele/catkin_ws/src/metapackages/homework1_test/test/BestParams.txt", ios::app);
	file.open ("/home/michele/catkin_ws/src/metapackages/homework1_test/test/ALLParams.txt", ios::app);
	for(double a = 2.0; a > 0; ){
		for(double b = 2.0; b > 0; ){
			for(double c = 4.0; c > 1.0; ){
				for(double d = 7.0; d > 4.0; ){
					for(double e = 5.0; e > 3.0; ){
						for(double f = 7.0; f > 3.0; ){
							model_ss_ = (float)(0.01 	+ 1.0);
							scene_ss_ = (float)(0.01 	+ 1.0);
							rf_rad_ = (float)(0.01 		+ c);
							descr_rad_ = (float)(0.01 	+ d);
							cg_size_ = (float)(0.01 	+ e);
							cg_thresh_ = (float)(0.01 	+ f);

							file << "TEST NUMBER " << count << "     PARAMS :"	<< endl;
							file <<	"     model_ss_     " 	<<	model_ss_ 	<< endl;
							file << "     scene_ss_     "	<<	scene_ss_ 	<< endl;
							file << "     rf_rad_       " 	<< 	rf_rad_  	<< endl;
							file << "     descr_rad_    " 	<< 	descr_rad_	<< endl;
							file << "     cg_size_      " 	<< 	cg_size_	<< endl;
							file << "     cg_thresh_    " 	<< cg_thresh_ 	<< endl;
							
							Correspondence_Grouping(cloud_filtered);

							count++;
							f = f - 1;
						}
						e = e - 0.5;
					}
					d = d - 0.5;
				}
				c = c - 0.5;
			}
			b = b - 0.5;
		}
		a = a - 0.5;
	}
	/**/


	if(argc_g > 1){
		model_ss_ = stof(argv_g[0]);
		scene_ss_ = stof(argv_g[1]);
		rf_rad_ = stof(argv_g[2]);
		descr_rad_ = stof(argv_g[3]);
		cg_size_ = stof(argv_g[4]);
		cg_thresh_ = stof(argv_g[5]);
	}else{
		model_ss_ = (float)0.03;
		scene_ss_ = (float)0.01;
		rf_rad_ = (float)1.1;
		descr_rad_ = (float)6.15;
		cg_size_ = (float)2.12;
		cg_thresh_ = (float)5.0;
	}

	Correspondence_Grouping(cloud_filtered);

}


int main(int argc, char **argv)
{	
	argc_g = argc;
	for (int i = 1; i < argc; i++){
        argv_g[i-1] = argv[i];
    }

	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Pcl_camera);
	//ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/hd/points", 1, Pcl_camera);
	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	ros::spin();

	file.close();
	file2.close();

	return 0;

}