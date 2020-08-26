#include "node_a.h"

using namespace Hm1;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

struct CloudStyle{
	double r;
	double g;
	double b;
	double size;

	CloudStyle (double r,
				double g,
				double b,
				double size) :
		r (r),
		g (g),
		b (b),
		size (size)
	{
	}
};

struct CompareIds{
	CompareIds(int x){ this->x = x;}
	bool operator() (int y){
		return y == x;
	}
	int x;
};

CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);
CloudStyle style_black (0.0, 0.0, 0.0, 2.0);

//Algorithm params
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (false);
bool use_hough_ (false);

bool hv_detect_clutter_ (true);

float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

int distanceThrs = 15, pointColorThrs = 30, regionColorThrs = 15;
float z_plane = 1.0;

std::vector<std::tuple<pcl::PointCloud<PointType>::ConstPtr, int>> registered_instances_tuple;

int argc_g; 
std::string argv_g[1];
int typeRun = 2;
std_msgs::String link_tf; 
std_msgs::String name_viewer; 

ros::Publisher pub, pub2;

void normalsVis (pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals){
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer viewer (name_viewer.data.c_str());
	viewer.setBackgroundColor (255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
	viewer.addPointCloud<PointType> (cloud, rgb, "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addPointCloudNormals<PointType, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
}

void simpleVis (pcl::PointCloud<PointType>::ConstPtr cloud){
  	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer (name_viewer.data.c_str());
	viewer.setBackgroundColor (255, 255, 255);
	viewer.addPointCloud<PointType> (cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
}

pcl::PointCloud<PointType>::Ptr rotoTraslatePCD(const pcl::PointCloud<PointType>::ConstPtr &cloud){
	PointType minPt, maxPt;
  	pcl::getMinMax3D (*cloud, minPt, maxPt);
	
	Eigen::Affine3f trasform = Eigen::Affine3f::Identity();
	if(minPt.z > z_plane){
		trasform.translation() << 0.0, 0.0, (z_plane - minPt.z); 
	}else{
		trasform.translation() << 0.0, 0.0, (z_plane - minPt.z);
	}

	//trasform.rotate(Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitZ()));
	trasform.matrix();

	pcl::PointCloud<PointType>::Ptr temp_cloud (new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*cloud, *temp_cloud, trasform);

	return temp_cloud;			
}

float findZPlane(const pcl::PointCloud<PointType>::ConstPtr &cloud){
	PointType minPt, maxPt;
  	pcl::getMinMax3D (*cloud, minPt, maxPt);
	ROS_INFO("Minimum Z Point of Plan Segmentation : %f", minPt.z);
	return minPt.z;
}

int ChooseColorMesh(const pcl::PointCloud<PointType>::ConstPtr &cloud, int i_cloud){
	int r = 0, g = 0, b = 0;
	int size_cloud = cloud->points.size();
	
	for(int i = 0; i < cloud->points.size(); i++){
		std::vector<int> rgb_temp_vector = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
		std::vector<int> rgb_vector = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
		std::sort(rgb_vector.begin(),rgb_vector.end(), std::greater<int>());
		if((rgb_vector[0] - rgb_vector[1] > 35) && (rgb_vector[0] - rgb_vector[2] > 35)){

			std::vector<int>::iterator it = std::find(rgb_temp_vector.begin(), rgb_temp_vector.end(), rgb_vector[0]);
			int index = std::distance(rgb_temp_vector.begin(), it);
			if(index == 0)
				r += cloud->points[i].r * 10;
			if(index == 1)
				g += cloud->points[i].g * 10;

		}else if((rgb_vector[0] - rgb_vector[1] < 35) && (rgb_vector[0] - rgb_vector[2] > 35)){
			r += cloud->points[i].r * 10;
			g += cloud->points[i].g * 10;
		}else{
			r += cloud->points[i].r;
			b += cloud->points[i].b;
			g += cloud->points[i].g;
		}
	}

	ROS_INFO("%d cloud, rgb color : (%d, %d, %d) ", i_cloud, r,g,b);
	
	std::vector<int> rgb_temp_vector = {r, g, b};
	std::vector<int> rgb_vector = {r, g, b};
	std::sort(rgb_vector.begin(),rgb_vector.end(), std::greater<int>());
	if((rgb_vector[0] - rgb_vector[1] > 500000) && (rgb_vector[0] - rgb_vector[2] > 500000)){								// find red or green

		std::vector<int>::iterator it = std::find(rgb_temp_vector.begin(), rgb_temp_vector.end(), rgb_vector[0]);
		int index = std::distance(rgb_temp_vector.begin(), it);
		if(index == 0){
			ROS_INFO("%d ........... Red ...........",i_cloud);
			return 0;
		}
		if(index == 1){
			ROS_INFO("%d ........... Green ...........",i_cloud);
			return 1;
		}
	}else if((rgb_temp_vector[0] - rgb_temp_vector[2] > 500000) && (rgb_temp_vector[1] - rgb_temp_vector[2] > 500000)){ 	// find yellow	
		ROS_INFO("%d ........... Yellow ...........",i_cloud);
		return 3;
	}else if((rgb_temp_vector[0] - rgb_temp_vector[1] < 500000) && (rgb_temp_vector[0] - rgb_temp_vector[2] < 500000)){    // find blue
		ROS_INFO("%d ........... Blue ...........",i_cloud);
		return 2;
	}else{		
		ROS_INFO("%d ........... Try all possibility ...........",i_cloud);
		return 4;
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

	for (std::size_t i = 0; i < cloud->size (); ++i){
		if (! std::isfinite ((*cloud)[i].x)){
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2){
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0){
		res /= n_points;
	}
	return res;
}

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &sceneConst, std::string filename, int i_cloud, float params[]){
	std::ofstream result;
	std::size_t posI = filename.find("meshes/");
	std::string sTemp = filename.substr(posI+7);
	std::string nameMesh = sTemp.substr(0, sTemp.length()-4);
	std::string namefile = "/home/michele/catkin_ws/src/metapackages/homework1_test/test/CG_" + std::to_string(i_cloud) + "_" + nameMesh.c_str() + ".txt"; 
	result.open (namefile);
	result << " ---- " << filename << " ---- " << "\n\n";

	pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());

	*scene = *sceneConst;

	if (pcl::io::loadPCDFile<PointType> (filename, *model) == -1){
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
	float model_ss_ = params[0];
	float scene_ss_ = params[1];
	float rf_rad_ = params[2];
	float descr_rad_ = params[3];
	float cg_size_ = params[4];
	float cg_thresh_ = params[5];

	int icp_max_iter_ (5);
	float icp_corr_distance_ (0.005f);
	float hv_clutter_reg_ = params[6];
	float hv_inlier_th_ = params[7];
	float hv_occlusion_th_ = params[8];
	float hv_rad_clutter_ = params[9];
	float hv_regularizer_ = params[10];
	float hv_rad_normals_ = params[11];

	result << "Model resolution:               " << resolution << "\n";
	result << "Model sampling size:            " << model_ss_ << "\n";
	result << "Scene sampling size:            " << scene_ss_ << "\n";
	result << "LRF support radius:             " << rf_rad_ << "\n";
	result << "SHOT descriptor radius:         " << descr_rad_ << "\n";
	result << "Clustering bin size:            " << cg_size_ << "\n";
	result << "Clustering bin size thresh:     " << cg_thresh_ << "\n";
	result << "Clutter Regularizer size:       " << hv_clutter_reg_ << "\n";
	result << "Inlier threshold size:          " << hv_inlier_th_ << "\n";
	result << "Occlusion threshold size:       " << hv_occlusion_th_ << "\n";
	result << "Clutter radius:                 " << hv_rad_clutter_ << "\n";
	result << "Regularizer value:              " << hv_regularizer_ << "\n";

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
	result << "Model total points: " << model->size () << " -- Selected Keypoints: " << model_keypoints->size () << "\n";

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	result << "Scene total points: " << scene->size () << " -- Selected Keypoints: " << scene_keypoints->size () << "\n";

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

	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);
	std::vector<int> model_good_keypoints_indices;
	std::vector<int> scene_good_keypoints_indices;

	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at (i).descriptor[0])){  //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.45f){
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
			model_good_keypoints_indices.push_back (corr.index_query);
			scene_good_keypoints_indices.push_back (corr.index_match);
		}
	}
	result << "model_good_keypoints_indices selected: " << model_good_keypoints_indices.size() << "\n";
	result << "scene_good_keypoints_indices selected: " << scene_good_keypoints_indices.size() << "\n";

	pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_good_kp (new pcl::PointCloud<PointType> ());
	pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);
	result << "Correspondences found: " << model_scene_corrs->size () << "\n";

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
	result << "Model instances found: " << rototranslations.size () << "\n";	

	/**
   	* Generates clouds for each instances found 
   	*/
  	std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		instances.push_back (rotated_model);
	}

	/**
	 * ICP
	 */
	std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
	result << "--------- ICP ---------" << "\n";
	for (std::size_t i = 0; i < rototranslations.size (); ++i){
		pcl::IterativeClosestPoint<PointType, PointType> icp;
		icp.setMaximumIterations (icp_max_iter_);
		icp.setMaxCorrespondenceDistance (icp_corr_distance_);
		icp.setInputTarget (scene);
		icp.setInputSource (instances[i]);
		pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
		icp.align (*registered);
		registered_instances.push_back (registered);
		result << "Instance " << i << "\n";
		if (icp.hasConverged ()){
			result << "Aligned!" << "\n";
		}
		else{
			result << "Not Aligned!" << "\n";
		}
	}
	result << "-----------------" << "\n\n";

	/**
	 * Hypothesis Verification
	 */
	result << "--- Hypotheses Verification ---" << "\n";
	std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

	pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;
	
	ROS_INFO("----- cloud : %d %s - instances : %d -----", i_cloud, nameMesh.c_str(), registered_instances.size());
	if(registered_instances.size() > 1){
		
		GoHv.setSceneCloud (scene);  // Scene Cloud
		GoHv.addModels (registered_instances, true);  //Models to verify
		GoHv.setResolution (resolution);
		GoHv.setInlierThreshold (hv_inlier_th_);
		GoHv.setOcclusionThreshold (hv_occlusion_th_);
		GoHv.setRegularizer (hv_regularizer_);
		GoHv.setRadiusClutter (hv_rad_clutter_);
		GoHv.setClutterRegularizer (hv_clutter_reg_);
		GoHv.setDetectClutter (hv_detect_clutter_);
		GoHv.setRadiusNormals (hv_rad_normals_);

		GoHv.verify ();
		GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

		for (int i = 0; i < hypotheses_mask.size (); i++){
			if (hypotheses_mask[i]){
				result << "Instance " << i << " is GOOD! <---\n";
			}
			else{
				result << "Instance " << i << " is bad!" << "\n";
			}
		}
	}
	result << "-------------------------------";

	bool findOne = false;
	for (std::size_t i = 0; i < registered_instances.size (); ++i){
		if (hypotheses_mask.size() != 0 && hypotheses_mask[i] ){
			findOne = true;
			std::tuple<pcl::PointCloud<PointType>::ConstPtr, int> temp_tuple;
			temp_tuple = std::make_tuple(registered_instances[i], i_cloud);
			registered_instances_tuple.push_back(temp_tuple);
			result.close();
		}
	}

	if(!findOne){
		result << "------ 0 model found for mesh : " << i_cloud << ", retry with new params ------";
		params[4] = cg_size_ - 0.002;
		params[7] = hv_inlier_th_ + 0.002;
		Correspondence_Grouping(scene, filename, i_cloud, params);
	}
}

std::vector<pcl::PointCloud<PointType>::Ptr> Euclidean_cluster_extraction(const pcl::PointCloud<PointType>::ConstPtr &cloud_filtered){

  	pcl::PointCloud <pcl::PointXYZRGB>::Ptr temp1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud_filtered, *temp1);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (temp1);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud (temp1);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (distanceThrs);			//25
	reg.setPointColorThreshold (pointColorThrs);		//27 - default 1225
	reg.setRegionColorThreshold (regionColorThrs);		//27 - default 10
	reg.setMinClusterSize (1500);
	reg.setMaxClusterSize (25000);
	reg.extract (cluster_indices);

  	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::PointCloud <PointType>::Ptr color_cloud (new pcl::PointCloud<PointType>);
	pcl::copyPointCloud(*colored_cloud, *color_cloud);
	/** /
	pcl::visualization::PCLVisualizer viewer3 ("Cloud Filtered and Extracted");
	viewer3.addCoordinateSystem(0.3);
	viewer3.addPointCloud (color_cloud, "cluster_max");
	
	while (!viewer3.wasStopped ()){
		viewer3.spinOnce ();
	}
	/**/

	std::vector<pcl::PointCloud<PointType>::Ptr> models_cloud_vector;

	int j = 0;
	for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);

		ROS_INFO("cloud_vector size : %d, indices number : %d", models_cloud_vector.size(), it->indices.size());

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
			cloud_temp->points.push_back (cloud_filtered->points[*pit]);
		}
		
		models_cloud_vector.push_back(cloud_temp);
		
		models_cloud_vector[j]->width = models_cloud_vector[j]->points.size ();
		models_cloud_vector[j]->height = 1;
		models_cloud_vector[j]->is_dense = true;
		j++;
	}
	
	return models_cloud_vector;
}	

double round_to_n_digits(double x, int n)
{ 
       char buff[32];

       sprintf(buff, "%.*g", n, x);

       return atof(buff);
}

std::vector<pcl::PointCloud<PointType>::Ptr> remove_plane(const pcl::PointCloud<PointType>::ConstPtr &cloud, 
					pcl::SACSegmentation<PointType> seg, pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg2,
					pcl::ExtractIndices<PointType> extract, pcl::ExtractIndices<PointType> extract2, 
					pcl::NormalEstimation<PointType, pcl::Normal> ne){
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_outliers (new pcl::PointCloud<PointType>);	
	pcl::PointCloud<PointType>::Ptr cloud_inliers (new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0){
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return {};
	}

	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.filter (*cloud_temp);

	ne.setInputCloud (cloud_temp);
	ne.compute (*cloud_normals);

	name_viewer.data = "Cloud Temp + Normals"; normalsVis(cloud_temp, cloud_normals);

	seg2.setInputCloud (cloud_temp);
	seg2.setInputNormals (cloud_normals);
	seg2.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0){
		PCL_ERROR ("Could not estimate a perpendicular model for the given dataset.");
		return {};
	}

	extract2.setInputCloud (cloud_temp);
	extract2.setIndices (inliers);
	extract2.setNegative (true);				// Extract the outliers
	extract2.filter (*cloud_outliers);			// cloud_outliers contains everything but the plane

	name_viewer.data = "Cloud Outliers"; simpleVis(cloud_outliers);

	extract.setInputCloud (cloud_temp);
	extract.setIndices (inliers);
	extract.setNegative (false);			
	extract.filter (*cloud_inliers);			// cloud_inliers contains only the plane
	
	name_viewer.data = "Cloud Inliers"; simpleVis(cloud_inliers);

	return{cloud_outliers, cloud_inliers};
}

pcl::PointCloud<PointType>::Ptr Plan_segmentation(const pcl::PointCloud<PointType>::ConstPtr &cloud){
	
	pcl::PointCloud<PointType>::Ptr cloud_outliers (new pcl::PointCloud<PointType>);	
	pcl::PointCloud<PointType>::Ptr cloud_inliers (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_outliers_filtered (new pcl::PointCloud<PointType>);
	std::vector<pcl::PointCloud<PointType>::Ptr> out_in_cloud;

	// Create the segmentation object
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMaxIterations(1000);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	
	pcl::ExtractIndices<PointType> extract;
	pcl::ExtractIndices<PointType> extract2;
	extract.setNegative (true);			
	
	// Estimate point normals
	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
	ne.setSearchMethod (tree);
	ne.setKSearch (50);

	pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg2; 
	seg2.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg2.setMaxIterations(100);
	seg2.setMethodType (pcl::SAC_RANSAC);
	seg2.setDistanceThreshold (0.01);

	pcl::copyPointCloud(*cloud, *cloud_outliers);

	PointType PtInPlane;
	//PtInPlane.x = -0.22833 ; PtInPlane.y = -0.546301 ; PtInPlane.z = 1.99363;
	PtInPlane.x = -0.22 ; PtInPlane.y = -0.54 ; PtInPlane.z = 1.99;
	bool isNotIn = false;
	do{

		// Iterate and remove all possible plane 
		out_in_cloud = remove_plane(cloud_outliers, seg, seg2, extract, extract2, ne);
		if(out_in_cloud.size() > 1){
			cloud_outliers = out_in_cloud[0];
			cloud_inliers = out_in_cloud[1];

			for(size_t i = 0; i < cloud_inliers->points.size(); i++)
				if(round_to_n_digits(cloud_inliers->points[i].x, 2) == PtInPlane.x){
					ROS_INFO("--- Contain X ---:         %f", cloud_inliers->points[i].x);
					if(round_to_n_digits(cloud_inliers->points[i].y, 2) == PtInPlane.y){
						ROS_INFO("--- Contain Y ---:         %f", cloud_inliers->points[i].y);
						if(round_to_n_digits(cloud_inliers->points[i].z, 2) == PtInPlane.z){
							ROS_INFO("--- Contain Y ---:         %f", cloud_inliers->points[i].z);
							isNotIn = false;
						}
					}
				}
		}
		

	}while(isNotIn);		//out_in_cloud.size() != 0);					

	PointType minPt, maxPt;
  	pcl::getMinMax3D (*cloud_inliers, minPt, maxPt);
	
	ROS_INFO("---------MIN X----------:         %f", minPt.x);
	ROS_INFO("---------MAX X----------:         %f", maxPt.x);
	ROS_INFO("---------MIN Y----------:         %f", minPt.y);
	ROS_INFO("---------MAX Y----------:         %f", maxPt.y);
	ROS_INFO("---------MIN Z----------:         %f", minPt.z);
	ROS_INFO("---------MAX Z----------:         %f", maxPt.z);

	pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment2 (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment3 (new pcl::PointCloud<PointType>);
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;
	pcl::PassThrough<PointType> pass3;
	pass.setInputCloud (cloud_outliers);
	pass.setFilterFieldName ("x");	
	pass.setFilterLimits (minPt.x, maxPt.x);
	//pass.filter (*cloud_segment);

	// Create the filtering object in Y
	pass2.setInputCloud (cloud_segment);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (minPt.y, maxPt.y);
	//pass2.filter (*cloud_segment2);

	// Create the filtering object in Y

	pass3.setInputCloud (cloud_segment2);
	pass3.setFilterFieldName ("z");	
	pass3.setFilterLimits (minPt.z - 0.5, maxPt.z + 0.5);
	//pass3.filter (*cloud_segment3);

	// Create the filtering object
  	pcl::StatisticalOutlierRemoval<PointType> sor;
  	sor.setInputCloud (cloud_outliers);//cloud_segment3);
  	sor.setMeanK (50);
  	sor.setStddevMulThresh (1.2);
	sor.filter (*cloud_outliers_filtered);
	

	name_viewer.data = "Cloud Outliers Filtered"; simpleVis(cloud_outliers_filtered);

	return cloud_outliers_filtered;
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){
	

	// *************************************** Trasformation from camera_link frame to world frame of MScloud *************************
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	sensor_msgs::PointCloud2 cloud_world, output, output2;

    //ros::Rate rate(30.0);
	ros::Duration timeout(400.0);
	try {
		if(typeRun == 1){
			transformStamped = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time::now(), timeout);
			tf2::doTransform(*MScloud, cloud_world, transformStamped);
		}else{
			transformStamped = tfBuffer.lookupTransform(link_tf.data.c_str(), "camera_link", ros::Time::now(), timeout);
			//transformStamped.header.frame_id = "world";
			tf2::doTransform(*MScloud, cloud_world, transformStamped);
		}

	} catch (tf2::TransformException &ex) {
		ROS_INFO("Error Trasformation...%s",ex.what());
	}
	ROS_INFO("cloud_world size : %d", cloud_world.height * cloud_world.row_step);
	
	if(cloud_world.height * cloud_world.row_step == 0){
		ROS_INFO("TF_OLD_DATA ignoring data... waiting next frame!");
		return;
	}

	// *************************************** Put Trasformed_PointCloud on PCL::PointCloud *************************
	pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment2 (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
	std::vector<pcl::PointCloud<PointType>::Ptr> cloud_extracted;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_triang (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<PointType>::Ptr cloud_final (new pcl::PointCloud<PointType>);

	
	//pcl::PCLPointCloud2 cloud_pcl_world;
	//pcl_conversions::toPCL(cloud_world, cloud_pcl_world);
    //pcl::fromPCLPointCloud2(cloud_pcl_world, *temp_cloud);

	//pcl::fromROSMsg(*MScloud.get(), *temp_cloud.get());
	//pcl::fromROSMsg(*cloud_world, *temp_cloud);
	//pcl::toROSMsg(*temp_cloud, output);
	//pub.publish(output);

	
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_world,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	ROS_INFO("temp_cloud size : %d", temp_cloud->size());
	name_viewer.data = "Cloud Temp"; simpleVis(temp_cloud);

	/*
	// *************************************** Filter on box X,Y *************************
	// Create the filtering object in X
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;
	pass.setInputCloud (temp_cloud);
	pass.setFilterFieldName ("x");
	if(typeRun == 0){
		pass.setFilterLimits (-0.55, +0.5);
	}else if(typeRun == 1){
		pass.setFilterLimits (-0.4, 0.7);	
	}else{
		pass.setFilterLimits (-1.0, +1.0);
		//pass.setFilterLimits (-3.6, +3.6);
	}
	pass.filter (*cloud_segment);

	// Create the filtering object in Y
	pass2.setInputCloud (cloud_segment);
	pass2.setFilterFieldName ("y");
	if(typeRun == 0){
		pass2.setFilterLimits (-0.97, -0.2);
	}else if(typeRun == 1){
		pass2.setFilterLimits (-1.42, -0.7);	
	}else{
		pass2.setFilterLimits (-2.2, +2.2);
		//pass2.setFilterLimits (-4.0, +4.0);
	}
	pass2.filter (*cloud_segment2);
	
	ROS_INFO("----- Semi cutting table done! -----");
	/**/
	// *************************************** Filter Using Segmentation  *************************
	  
	if(temp_cloud->size() != 0){

		cloud_filtered = Plan_segmentation(temp_cloud);

		ROS_INFO("----- Plan segmentation done! -----");
		
		simpleVis(cloud_filtered);

		if(typeRun == 0) {
			z_plane = findZPlane(cloud_filtered);					// REAL works because good clear of isolated points
		}

		cloud_extracted = Euclidean_cluster_extraction(cloud_filtered);
		
		ROS_INFO("----- Region Growing RGB done! -----");

	}else{
		ROS_INFO("Cloud empty... cannot segmentation...");
		return;
	}
	/*
	pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
	viewer.removeAllShapes();
	viewer.removeAllPointClouds();
	viewer.setBackgroundColor (1, 1, 1);
	viewer.addPointCloud (cloud_segment2, "scene_cloud");

	// POSSIBLE TRIANGLE : 0.001 0.001 0.03 0.03 0.01 10.0 5.0 0.18 0.01 0.03 3.0 0.05
	// POSSIBLE CUBE 	 : 0.001 0.001 0.03 0.03 0.03 10.0 5.0 0.3 0.01 0.03 3.0 0.05
	// POSSIBLE HEXAGON  : 0.001 0.001 0.03 0.03 0.03 10.0 5.0 0.3 0.01 0.03 3.0 0.05
	
	float paramsHex[12] = 	{0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.3, 0.01, 0.03, 3.0, 0.05}, 
		  paramsCube[12] = 	{0.001, 0.001, 0.03, 0.03, 0.035, 10.0, 5.0, 0.29, 0.01, 0.03, 3.0, 0.05}, 
		  paramsTrian[12] = {0.001, 0.001, 0.03, 0.03, 0.035, 10.0, 5.0, 0.35, 0.01, 0.03, 3.0, 0.05};
	
	std::vector<std::thread> threads;
	for(int i = 0; i < cloud_extracted.size(); i++){
		int color = 5; 	//default condition
		std::string filename;
		color = ChooseColorMesh(cloud_extracted[i],i);
		switch (color){
			case (0):		// red
				if(typeRun == 2){
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}else{
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}
			break;
			case (1):		// green
				if(typeRun == 2){
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}else{
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}
			break;
			case (2):		// blue
				if(typeRun == 2){
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
				}else{
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
				}
			break;
			case (3):		// yellow
				if(typeRun == 2){
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));
				}else{
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));	
				}
			break;
			case (4):		// try all mesh
				if(typeRun == 2){
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));
				}else{
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_all_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));	
				}
			break;
			default:
				ROS_INFO("Cannot continue with Correspondence Grouping!!");
				return;
			break;
		}
	}
	for (std::thread & th : threads){
		if (th.joinable()){
			th.join();
		}
	}

	ROS_INFO(" -------------------------- Thread created : %d", threads.size());
	
	std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances_global;
	std::vector<int> i_instances_global;
	for(std::tuple<pcl::PointCloud<PointType>::ConstPtr, int> t : registered_instances_tuple){
		registered_instances_global.push_back(std::get<0>(t));
		i_instances_global.push_back(std::get<1>(t));
		ROS_INFO("Indices of global pointcloud found : %d", std::get<1>(t));
	}
	ROS_INFO("registered_instances size : %d --- instances :%d", registered_instances_global.size(),i_instances_global.size());
	
	ROS_INFO("------- Finding multi mesh on the same model -------");
	std::vector<int> iToStay;
	for(int i = 0; i < cloud_extracted.size(); i++){
		std::vector<int> temp_index;
		std::vector<int> temp_index_hv;
		std::vector<int>::iterator iter = i_instances_global.begin();
		while ((iter = std::find_if(iter, i_instances_global.end(), CompareIds(i))) != i_instances_global.end()){
			temp_index.push_back(std::distance(i_instances_global.begin(), iter));
			iter++;
		}
		
		if(temp_index.size() > 1){
			std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
			for(int index : temp_index){
				registered_instances.push_back(registered_instances_global[index]);
			}

			float setInTh = 0.3;
			bool findMod = false;
			while(!(temp_index_hv.size() == 1) && setInTh < 0.4){
				findMod = false;
				setInTh += 0.01;
				temp_index_hv.clear();
				// Hypothesis Verification
				ROS_INFO( "--- Hypotheses Verification for model --- setInlierThreshold : %f", setInTh);
				std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

				pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

				GoHv.setSceneCloud (cloud_extracted[i]);  // Scene Cloud
				GoHv.addModels (registered_instances, true);  //Models to verify
				GoHv.setResolution (0.003f); // resolution
				GoHv.setInlierThreshold (setInTh);
				GoHv.setOcclusionThreshold (0.01f);
				GoHv.setRegularizer (3.0);
				GoHv.setRadiusClutter (0.03f);
				GoHv.setClutterRegularizer (5.0f);
				GoHv.setDetectClutter (hv_detect_clutter_);
				GoHv.setRadiusNormals (0.05f);

				GoHv.verify ();
				GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

				for (int i = 0; i < hypotheses_mask.size (); i++){
					if (hypotheses_mask[i]){
						findMod = true;
						ROS_INFO( "Instance %d is GOOD! <---", i );
						temp_index_hv.push_back(temp_index[i]);
					}
					else{
						ROS_INFO( "Instance %d is bad!", i );
					}
				}
				ROS_INFO( "-------------------------------" );
			}
			if(!findMod){
				ROS_INFO("Any model passed hyphotesis verification test.. take random one! -- cloud : %d", i);
				iToStay.push_back(temp_index[rand() % temp_index.size()]);
			}else if(temp_index_hv.size() > 1){
				ROS_INFO("A lot of model passed hyphotesis verification test.. take a random one! -- cloud : %d", i);
				iToStay.push_back(temp_index_hv[rand() % temp_index_hv.size()]);
			}else{
				ROS_INFO("Only 1 model passed hyphotesis verification test.. take it! -- cloud : %d", i);
				iToStay.push_back(temp_index_hv[0]);
			}
		}else{
			ROS_INFO("Found only 1 model of cloud : %d", i);
			iToStay.push_back(temp_index[0]);
		}
	}

	ROS_INFO("registered_instances_global size %d", registered_instances_global.size());
	for (std::size_t i = 0; i < registered_instances_global.size (); ++i){
		if(std::find(iToStay.begin(), iToStay.end(), i) != iToStay.end()){
			ROS_INFO("Pointcloud added : %d", i);

			// RotoTranslate models found
			pcl::PointCloud<PointType>::Ptr cloud_rotoTrans (new pcl::PointCloud<PointType>);
			*cloud_rotoTrans = *registered_instances_global[i];
			//cloud_rotoTrans = rotoTraslatePCD(registered_instances_global[i]);

			CloudStyle clusterStyle = style_black;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (cloud_rotoTrans, clusterStyle.r, clusterStyle.g, clusterStyle.b);
			viewer.addPointCloud (cloud_rotoTrans, instance_color_handler,  std::to_string(i));
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, std::to_string(i));
		}
	}
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	/**/
}


int main(int argc, char **argv){	
	
	argc_g = argc;
	
	for (int i = 1; i < argc; i++){
        argv_g[i-1] = argv[i];
    }
	link_tf.data = "camera_link";
	if(argc_g > 0){
		link_tf.data = argv_g[0].c_str();
	}

	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ROS_INFO("---------- Parameters list already set ----------");
	ROS_INFO("     typeRun : %d", typeRun);
	ROS_INFO("     show_keypoints_");
	ROS_INFO("     show_correspondences_");
	ROS_INFO("     use_cloud_resolution_");
	ROS_INFO("     use_hough_");
	ROS_INFO("     model_ss_");
	ROS_INFO("     scene_ss_");
	ROS_INFO("     rf_rad_");
	ROS_INFO("     descr_rad_");
	ROS_INFO("     cg_size_");
	ROS_INFO("     cg_thresh_");
	ROS_INFO("     hv_clutter_reg_");
	ROS_INFO("     hv_inlier_th_");
	ROS_INFO("     hv_occlusion_th_");
	ROS_INFO("     hv_rad_clutter_");
	ROS_INFO("     hv_regularizer_");
	ROS_INFO("     hv_rad_normals_");
	ROS_INFO("     region_growing_setDistanceThreshold");
	ROS_INFO("     region_growing_setPointColorThreshold");
	ROS_INFO("     region_growing_setRegionColorThreshold");
	ROS_INFO("     angular_resolution");
	ROS_INFO("     setUnseenToMaxRange");
	ROS_INFO("     LINK CHOOSEN: %s\n", link_tf.data.c_str());
	ROS_INFO("-------------------------------------------");

	
	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	pub2 = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud_inliers", 1);

	ros::Subscriber sub;
	if(typeRun == 0){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/hd/points", 1, Pcl_camera);		// REAL
	}else if(typeRun == 1){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect_cloud", 1, Pcl_camera);			// REAL-SIMULATED
	}else if(typeRun == 2){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Pcl_camera);		// SIMULATED
	}	
	
	ros::spin();

	return 0;

}

