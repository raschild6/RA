#include <sstream>
#include <iostream>
#include <thread> 
#include <fstream>
#include <stdlib.h>
#include <tuple>
#include <cstdlib>

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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
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
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


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

struct CustomPointXYZ{
	float x;
	float y;
	float z;

	bool operator<(const CustomPointXYZ& cPoint) const{
        return (x < cPoint.x && y < cPoint.y && z < cPoint.z);
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

float resolution (0.003f);
float model_ss_ (0.001f);
float scene_ss_ (0.001f);
float rf_rad_ (0.03f);
float descr_rad_ (0.03f);
float cg_size_ (0.03f);
float cg_thresh_ (10.0f);

int icp_max_iter_ (5);
float icp_corr_distance_ (0.005f);
//float hv_resolution_ (0.005f);						//Used resolution calculated from Correspondence_Grouping
//float hv_occupancy_grid_resolution_ (0.01f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.3f);
float hv_occlusion_th_ (0.01f);
float hv_rad_clutter_ (0.03f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.05);
bool hv_detect_clutter_ (true);

float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

int SAMPLE_POINTS_ = 10000;

int distanceThrs = 20, pointColorThrs = 50, regionColorThrs = 20;
float z_plane = 1.0;


std::vector<std::tuple<pcl::PointCloud<PointType>::ConstPtr, int>> registered_instances_tuple;

int argc_g; 
std::string argv_g[17];
bool viewMod = false, use_triang = false;

int count = 1;
std::ofstream file,file2;
ros::Publisher pub, pub2;

int best2 = 100;


double uniform_deviate (int seed){
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
              		      float r1, float r2, Eigen::Vector3f& p){
	float r1sqr = std::sqrt (r1);
	float OneMinR1Sqr = (1 - r1sqr);
	float OneMinR2 = (1 - r2);
	a1 *= OneMinR1Sqr;
	a2 *= OneMinR1Sqr;
	a3 *= OneMinR1Sqr;
	b1 *= OneMinR2;
	b2 *= OneMinR2;
	b3 *= OneMinR2;
	c1 = r1sqr * (r2 * c1 + b1) + a1;
	c2 = r1sqr * (r2 * c2 + b2) + a2;
	c3 = r1sqr * (r2 * c3 + b3) + a3;
	p[0] = c1;
	p[1] = c2;
	p[2] = c3;
}

void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3f& p, bool calcNormal, Eigen::Vector3f& n, bool calcColor, Eigen::Vector3f& c){
	float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

	std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
	vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

	double A[3], B[3], C[3];
	vtkIdType npts = 0;
	vtkIdType *ptIds = nullptr;
	polydata->GetCellPoints (el, npts, ptIds);
	polydata->GetPoint (ptIds[0], A);
	polydata->GetPoint (ptIds[1], B);
	polydata->GetPoint (ptIds[2], C);
	if (calcNormal){
		// OBJ: Vertices are stored in a counter-clockwise order by default
		Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
		Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
		n = v1.cross (v2);
		n.normalize ();
	}
	float r1 = static_cast<float> (uniform_deviate (rand ()));
	float r2 = static_cast<float> (uniform_deviate (rand ()));
	randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
						float (B[0]), float (B[1]), float (B[2]),
						float (C[0]), float (C[1]), float (C[2]), r1, r2, p);

	if (calcColor){
		vtkUnsignedCharArray *const colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
		if (colors && colors->GetNumberOfComponents () == 3){
			double cA[3], cB[3], cC[3];
			colors->GetTuple (ptIds[0], cA);
			colors->GetTuple (ptIds[1], cB);
			colors->GetTuple (ptIds[2], cC);

			randomPointTriangle (float (cA[0]), float (cA[1]), float (cA[2]),
								float (cB[0]), float (cB[1]), float (cB[2]),
								float (cC[0]), float (cC[1]), float (cC[2]), r1, r2, c);
		}
		else
		{
			static bool printed_once = false;
			if (!printed_once)
				PCL_WARN ("Mesh has no vertex colors, or vertex colors are not RGB!");
			printed_once = true;
		}
	}
}

void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples, bool calc_normal, bool calc_color, pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud_out){
	polydata->BuildCells ();
	vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

	double p1[3], p2[3], p3[3], totalArea = 0;
	std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
	vtkIdType npts = 0, *ptIds = nullptr;
	std::size_t cellId = 0;
	for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); cellId++){
		polydata->GetPoint (ptIds[0], p1);
		polydata->GetPoint (ptIds[1], p2);
		polydata->GetPoint (ptIds[2], p3);
		totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
		cumulativeAreas[cellId] = totalArea;
	}

	cloud_out.points.resize (n_samples);
	cloud_out.width = static_cast<std::uint32_t> (n_samples);
	cloud_out.height = 1;

	for (std::size_t i = 0; i < n_samples; i++){
		Eigen::Vector3f p;
		Eigen::Vector3f n (0, 0, 0);
		Eigen::Vector3f c (0, 0, 0);
		randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n, calc_color, c);
		cloud_out.points[i].x = p[0];
		cloud_out.points[i].y = p[1];
		cloud_out.points[i].z = p[2];
		if (calc_normal){
			cloud_out.points[i].normal_x = n[0];
			cloud_out.points[i].normal_y = n[1];
			cloud_out.points[i].normal_z = n[2];
		}
		if (calc_color){
			cloud_out.points[i].r = static_cast<std::uint8_t>(c[0]);
			cloud_out.points[i].g = static_cast<std::uint8_t>(c[1]);
			cloud_out.points[i].b = static_cast<std::uint8_t>(c[2]);
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr FastTriangulation(const pcl::PointCloud<PointType>::ConstPtr &cloud_ptr){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
	*cloud_temp = *cloud_ptr;
	pcl::copyPointCloud(*cloud_temp, *cloud);

	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (10);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();



	vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
	pcl::io::mesh2vtk (triangles, polydata1);
	
	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
	triangleFilter->SetInputData (polydata1);
	triangleFilter->Update ();

	vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
	triangleMapper->Update ();
	polydata1 = triangleMapper->GetInput ();

	bool write_normals = true, write_colors = false;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	uniform_sampling (polydata1, SAMPLE_POINTS_, write_normals, write_colors, *cloud_1);

	return cloud_1;
}

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

bool equalPointXYZ (const CustomPointXYZ& cPoint1, const CustomPointXYZ& cPoint2){
	return (cPoint1.x == cPoint2.x && cPoint1.y == cPoint2.y && cPoint1.z == cPoint2.z);
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

	//*temp_cloud = *cloud;				// NOT USE in REAL
	return temp_cloud;			
}

float findZPlane(const pcl::PointCloud<PointType>::ConstPtr &cloud){
	PointType minPt, maxPt;
  	pcl::getMinMax3D (*cloud, minPt, maxPt);
	ROS_INFO("Minimum Z Point of Plan Segmentation : %f", minPt.z);
	return minPt.z;
}

int nCorrespondence(const pcl::PointCloud<PointType>::ConstPtr &cloud1, const pcl::PointCloud<PointType>::ConstPtr &cloud2){
	std::vector<CustomPointXYZ> point_vec1, point_vec2, equal_vec;
	for(int i = 0; i < cloud1->points.size(); i++){
		struct CustomPointXYZ cPoint = {cloud1->points[i].x, cloud1->points[i].y, cloud1->points[i].z};
		point_vec1.push_back(cPoint);
	} 
	for(int i = 0; i < cloud2->points.size(); i++){
		struct CustomPointXYZ cPoint = {cloud2->points[i].x, cloud2->points[i].y, cloud2->points[i].z};
		point_vec2.push_back(cPoint);
	}
	std::sort( point_vec1.begin(), point_vec1.end() );
	std::sort( point_vec2.begin(), point_vec2.end() );	

	std::set_intersection(point_vec1.begin(), point_vec1.end(),
							 point_vec2.begin(), point_vec2.end(), std::back_inserter(equal_vec), equalPointXYZ);
	
	ROS_INFO("number of exact correspondences found between original model and meshes : %d",  equal_vec.size());

	return equal_vec.size();
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

void Correspondence_Grouping (const pcl::PointCloud<PointType>::ConstPtr &sceneConst, std::string filename, int i_cloud){
	std::ofstream result;
	std::size_t posI = filename.find("meshes/");
	std::string sTemp = filename.substr(posI+7);
	std::string nameMesh = sTemp.substr(0, sTemp.length()-4);
	std::string namefile = "/home/michele/catkin_ws/src/metapackages/homework1_test/test/CG_" + std::to_string(i_cloud) + "_" + nameMesh.c_str() + ".txt"; 
	//ROS_INFO("Name of file created : %s", namefile.c_str());
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
	resolution = static_cast<float> (computeCloudResolution (model));
	if(use_cloud_resolution_){
		if (resolution != 0.0f){
			model_ss_   *= resolution;
			scene_ss_   *= resolution;
			rf_rad_     *= resolution;
			descr_rad_  *= resolution;
			cg_size_    *= resolution;
		}
	}

	// POSSIBLE TRIANGLE : 0.001 0.001 0.03 0.03 0.01 10.0 5.0 0.18 0.01 0.03 3.0 0.05
	// POSSIBLE CUBE 	 : 0.001 0.001 0.03 0.03 0.03 10.0 5.0 0.3 0.01 0.03 3.0 0.05
	// POSSIBLE HEXAGON  : 0.001 0.001 0.03 0.03 0.03 10.0 5.0 0.3 0.01 0.03 3.0 0.05

	if(nameMesh.compare("hexagon_2k") == 0){
		model_ss_ = (0.001f);
		scene_ss_ = (0.001f);
		rf_rad_ = (0.03f);
		descr_rad_ = (0.03f);
		cg_size_ = (0.03f);
		cg_thresh_ = (10.0f);
		hv_clutter_reg_ = (5.0f);
		hv_inlier_th_ = (0.3f);
		hv_occlusion_th_ = (0.01f);
		hv_rad_clutter_ = (0.03f);
		hv_regularizer_ = (3.0f);
		hv_rad_normals_ = (0.05f);

	}else if(nameMesh.compare("cube_2k") == 0){
		model_ss_ = (0.001f);
		scene_ss_ = (0.001f);
		rf_rad_ = (0.03f);
		descr_rad_ = (0.03f);
		cg_size_ = (0.03f);
		cg_thresh_ = (10.0f);
		hv_clutter_reg_ = (5.0f);
		hv_inlier_th_ = (0.3f);
		hv_occlusion_th_ = (0.01f);
		hv_rad_clutter_ = (0.03f);
		hv_regularizer_ = (3.0f);
		hv_rad_normals_ = (0.05f);

	}else if(nameMesh.compare("triangle_centered_2k") == 0){
		model_ss_ = (0.001f);
		scene_ss_ = (0.001f);
		rf_rad_ = (0.03f);
		descr_rad_ = (0.03f);
		cg_size_ = (0.025f);
		cg_thresh_ = (10.0f);
		hv_clutter_reg_ = (5.0f);
		hv_inlier_th_ = (0.2f);
		hv_occlusion_th_ = (0.01f);
		hv_rad_clutter_ = (0.03f);
		hv_regularizer_ = (3.0f);
		hv_rad_normals_ = (0.05f);

	}else{
		model_ss_ = (0.001f);
		scene_ss_ = (0.001f);
		rf_rad_ = (0.03f);
		descr_rad_ = (0.03f);
		cg_size_ = (0.02f);
		cg_thresh_ = (10.0f);
		hv_clutter_reg_ = (5.0f);
		hv_inlier_th_ = (0.3f);
		hv_occlusion_th_ = (0.01f);
		hv_rad_clutter_ = (0.03f);
		hv_regularizer_ = (3.0f);
		hv_rad_normals_ = (0.05f);
	}

	/*
	ROS_INFO ( "Use traingulation:              %d", use_triang);
	ROS_INFO ( "Model resolution:           	%f", resolution);
	ROS_INFO ( "Model sampling size:        	%f", model_ss_ );
	ROS_INFO ( "Scene sampling size:        	%f", scene_ss_ );
	ROS_INFO ( "LRF support radius:         	%f", rf_rad_ );
	ROS_INFO ( "SHOT descriptor radius:     	%f", descr_rad_ );
	ROS_INFO ( "Clustering bin size:        	%f", cg_size_);
	ROS_INFO ( "Clustering bin size thresh:    	%f", cg_thresh_);
	ROS_INFO ( "Clutter Regularizer size:   	%f", hv_clutter_reg_);
	ROS_INFO ( "Inlier threshold size:      	%f", hv_inlier_th_ );
	ROS_INFO ( "Occlusion threshold size:   	%f", hv_occlusion_th_ );
	ROS_INFO ( "Clutter radius:             	%f", hv_rad_clutter_ );
	ROS_INFO ( "Regularizer value:          	%f", hv_regularizer_ );
	/*
	ROS_INFO ( "xMin value:                     %f", xMin);
	ROS_INFO ( "xMax value:                     %f", xMax);
	ROS_INFO ( "yMin value:                     %f", yMin);
	ROS_INFO ( "yMax value:                     %f", yMax);
	*/

					result << "Use traingulation:              " << use_triang << "\n";
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
	//ROS_INFO ( "Model total points: %d ; Selected Keypoints: %d", model->size (), model_keypoints->size ());
	result << "Model total points: " << model->size () << " -- Selected Keypoints: " << model_keypoints->size () << "\n";

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.filter (*scene_keypoints);
	//ROS_INFO ( "Scene total points: %d ; Selected Keypoints: %d", scene->size (), scene_keypoints->size ());
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
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.75f){
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
			model_good_keypoints_indices.push_back (corr.index_query);
			scene_good_keypoints_indices.push_back (corr.index_match);
		}
	}
	//ROS_INFO("model_good_keypoints_indices selected : %d", model_good_keypoints_indices.size());
	//ROS_INFO("scene_good_keypoints_indices selected : %d", scene_good_keypoints_indices.size());
	result << "model_good_keypoints_indices selected: " << model_good_keypoints_indices.size() << "\n";
	result << "scene_good_keypoints_indices selected: " << scene_good_keypoints_indices.size() << "\n";

	pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr scene_good_kp (new pcl::PointCloud<PointType> ());
	pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

  	//ROS_INFO( "Correspondences found: %d", model_scene_corrs->size ());
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
	//ROS_INFO ("Model instances found: %d", rototranslations.size ());
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
	if (true){
		//ROS_INFO( "--------- ICP ---------" );
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
			//ROS_INFO( "Instance %d", i);
			result << "Instance " << i << "\n";
			if (icp.hasConverged ()){
				//ROS_INFO( "Aligned!" );
				result << "Aligned!" << "\n";
			}
			else{
				//ROS_INFO( "Not Aligned!" );
				result << "Not Aligned!" << "\n";
			}
		}
		//ROS_INFO( "-----------------" );
		//ROS_INFO("");
		result << "-----------------" << "\n\n";
	}

	/**
	 * Hypothesis Verification
	 */
	//ROS_INFO( "--- Hypotheses Verification ---" );
	result << "--- Hypotheses Verification ---" << "\n";
	std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

	pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

	if(registered_instances.size() != 0){
		
		GoHv.setSceneCloud (scene);  // Scene Cloud
		GoHv.addModels (registered_instances, true);  //Models to verify
		GoHv.setResolution (resolution);
		//GoHv.setResolutionOccupancyGrid (hv_occupancy_grid_resolution_);
		GoHv.setInlierThreshold (hv_inlier_th_);
		GoHv.setOcclusionThreshold (hv_occlusion_th_);
		GoHv.setRegularizer (hv_regularizer_);
		GoHv.setRadiusClutter (hv_rad_clutter_);
		GoHv.setClutterRegularizer (hv_clutter_reg_);
		GoHv.setDetectClutter (hv_detect_clutter_);
		GoHv.setRadiusNormals (hv_rad_normals_);

		GoHv.verify ();
		GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
		bool findMod = false;

		for (int i = 0; i < hypotheses_mask.size (); i++){
			if (hypotheses_mask[i]){
				viewMod = true;
				findMod = true;
				//ROS_INFO( "Instance %d is GOOD! <---", i );
				result << "Instance " << i << " is GOOD! <---\n";
			}
			else{
				//ROS_INFO( "Instance %d is bad!", i );
				result << "Instance " << i << " is bad!" << "\n";
			}
		}
	}
	//ROS_INFO( "-------------------------------" );
	result << "-------------------------------";

	/**
	 *  Visualization
	 * /
	if(viewMod){
		pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
		viewer.removeAllShapes();
		viewer.removeAllPointClouds();
		viewer.addPointCloud (scene, "scene_cloud");

		pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
		pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

		pcl::PointCloud<PointType>::Ptr off_model_good_kp (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

		if (show_keypoints_){
			CloudStyle modelStyle = style_white;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
			viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
		}

		if (show_keypoints_){
			CloudStyle goodKeypointStyle = style_violet;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler (off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
																											goodKeypointStyle.b);
			viewer.addPointCloud (off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler (scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
																											goodKeypointStyle.b);
			viewer.addPointCloud (scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
		}

		for (std::size_t i = 0; i < instances.size (); ++i){
			if (hypotheses_mask[i]){
				std::stringstream ss_instance;
				ss_instance << "instance_" << i;

				CloudStyle clusterStyle = style_red;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
				viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

				CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
				ss_instance << "_registered" << std::endl;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
																											registeredStyles.g, registeredStyles.b);
				viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
			}
		}

		while (!viewer.wasStopped ()){
			viewer.spinOnce ();
		}
	}
	/**/
	
	for (std::size_t i = 0; i < instances.size (); ++i){
		if (hypotheses_mask[i]){
			std::tuple<pcl::PointCloud<PointType>::ConstPtr, int> temp_tuple;
			temp_tuple = std::make_tuple(registered_instances[i], i_cloud);
			registered_instances_tuple.push_back(temp_tuple);
		}
	}
	result.close();
}

pcl::PointCloud<PointType>::Ptr Remove_from_condition(const pcl::PointCloud<PointType>::ConstPtr &cloud, PointType minPt, PointType maxPt ){
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
	
	// build the condition
    pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType> ());
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, maxPt.x)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, minPt.x)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, maxPt.y)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, minPt.y)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, maxPt.z)));
	range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, minPt.z)));
    
	// build the filter
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);	
    //condrem.setKeepOrganized(true);
    
	// apply filter
    condrem.filter (*cloud_filtered);
	
	return cloud_filtered;
}

std::vector<pcl::PointCloud<PointType>::Ptr> Euclidean_cluster_extraction(const pcl::PointCloud<PointType>::ConstPtr &cloud_filtered){
	//pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	//tree->setInputCloud (cloud_filtered);

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
	/**/
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

		/** /
		pcl::visualization::PCLVisualizer viewer5 ("Cloud Filtered and Extracted");
		viewer5.addCoordinateSystem(0.3);
		viewer5.addPointCloud (models_cloud_vector[j], "cluster_single");
		
		while (!viewer5.wasStopped ()){
			viewer5.spinOnce ();
		}
		/**/

		j++;
	}
	
	return models_cloud_vector;
}	

void Extract_border(const pcl::PointCloud<PointType>::ConstPtr &cloud_const){
	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType> ());
	*cloud = *cloud_const;

	pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

	point_cloud = *cloud; 

	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
															point_cloud.sensor_origin_[1],
															point_cloud.sensor_origin_[2])) *
						Eigen::Affine3f (point_cloud.sensor_orientation_);

	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	//float xMin = -5.0, xMax = +5.0, yMin = -5.0, yMax = +5.0;
	pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;   
	range_image.createFromPointCloud (point_cloud, pcl::deg2rad (0.9f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
									scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	
	range_image.setUnseenToMaxRange ();

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	viewer.addCoordinateSystem (1.0f, "global");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
	//viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");
	
	// -------------------------
	// -----Extract borders-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor (&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute (border_descriptions);
	
	// ----------------------------------
	// -----Show points in 3D viewer-----
	// ----------------------------------
	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
												veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
												shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
										& veil_points = * veil_points_ptr,
										& shadow_points = *shadow_points_ptr;
	for (int y=0; y< (int)range_image.height; ++y)
	{
		for (int x=0; x< (int)range_image.width; ++x)
		{
		if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]){
				border_points.points.push_back (range_image.points[y*range_image.width + x]);
				//ROS_INFO("BORDER_TRAIT__OBSTACLE_BORDER x,y = %d,%d",x,y);
		}
		if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT]){
				veil_points.points.push_back (range_image.points[y*range_image.width + x]);
				//ROS_INFO("BORDER_TRAIT__VEIL_POINT x,y = %d,%d",x,y);
		}
		if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER]){
				shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
				//ROS_INFO("BORDER_TRAIT__SHADOW_BORDER x,y = %d,%d",x,y);
			}
		}
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
	viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
	viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
	
	//-------------------------------------
	// -----Show points on range image-----
	// ------------------------------------
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
																			border_descriptions, "Range image with borders");
	// -------------------------------------
	
	
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped ())
	{
		range_image_borders_widget->spinOnce ();
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}
}

pcl::PointCloud<PointType>::Ptr Plan_segmentation(const pcl::PointCloud<PointType>::ConstPtr &cloud){

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_outliers (new pcl::PointCloud<PointType>);	
	pcl::PointCloud<PointType>::Ptr cloud_outliers_filtered (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_inliers (new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	

	// Create the segmentation object
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMaxIterations(100);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.0001);
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

	extract.setInputCloud (cloud_temp);
	extract.setIndices (inliers);
	extract.setNegative (false);			
	extract.filter (*cloud_inliers);
	
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	extract_normals.setNegative (true);
  	extract_normals.setInputCloud (cloud_normals);
  	extract_normals.setIndices (inliers);
  	extract_normals.filter (*cloud_normals2);

	/*
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_inliers, output);
	output.header.frame_id = "world";
	pub2.publish(output);
	*/
	//Extract_border(cloud_inliers);
	//cloud_outliers = Euclidean_cluster_extraction(cloud_inliers);

	PointType minPt, maxPt;
  	pcl::getMinMax3D (*cloud_inliers, minPt, maxPt);
	
	ROS_INFO("---------MIN X----------:         %f", minPt.x);
	ROS_INFO("---------MAX X----------:         %f", maxPt.x);
	ROS_INFO("---------MIN Y----------:         %f", minPt.y);
	ROS_INFO("---------MAX Y----------:         %f", maxPt.y);
	ROS_INFO("---------MIN Z----------:         %f", minPt.z);
	ROS_INFO("---------MAX Z----------:         %f", maxPt.z);

	//cloud_outliers_filtered = Remove_from_condition(cloud_outliers, minPt, maxPt);

	/** /
	pcl::PointCloud<PointType>::Ptr cloud_Eigen  (new pcl::PointCloud<PointType>);	
	
	cloud_Eigen->points.push_back(minPt);
	cloud_Eigen->points.push_back(maxPt);
	
	cloud_Eigen->width = cloud_Eigen->points.size ();
	cloud_Eigen->height = 1;
	cloud_Eigen->is_dense = true;

	pcl::visualization::PCLVisualizer viewer ("Cloud Filtered and Extracted");
	viewer.addCoordinateSystem(0.3);
	viewer.addPointCloud (cloud_inliers, "cloud_extracted");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> border_points_color_handler (cloud_Eigen, 0, 255, 0);
	viewer.addPointCloud<PointType> (cloud_Eigen, border_points_color_handler, "border points");
	
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	/**/


	pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment2 (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment3 (new pcl::PointCloud<PointType>);
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;
	pcl::PassThrough<PointType> pass3;
	pass.setInputCloud (cloud_outliers);
	pass.setFilterFieldName ("x");	
	pass.setFilterLimits (minPt.x, maxPt.x);
	pass.filter (*cloud_segment);

	// Create the filtering object in Y
	pass2.setInputCloud (cloud_segment);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (minPt.y, maxPt.y);
	pass2.filter (*cloud_segment2);

	// Create the filtering object in Y
	pass3.setInputCloud (cloud_segment2);
	pass3.setFilterFieldName ("z");	
	pass3.setFilterLimits (minPt.z, maxPt.z + 0.35);
	pass3.filter (*cloud_segment3);

	 // Create the filtering object
  	pcl::StatisticalOutlierRemoval<PointType> sor;
  	sor.setInputCloud (cloud_segment3);
  	sor.setMeanK (50);
  	sor.setStddevMulThresh (1.2);
	sor.filter (*cloud_outliers_filtered);

	/*
	pcl::visualization::PCLVisualizer viewer ("Cloud Filtered and Extracted");
	viewer.addCoordinateSystem(0.3);
	viewer.addPointCloud (cloud_outliers_filtered, "cloud_extracted");
	
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	*/
	return cloud_outliers_filtered;
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){
	/*
	if(argc_g > 1){
		model_ss_ = stof(argv_g[0]);
		scene_ss_ = stof(argv_g[1]);
		rf_rad_ = stof(argv_g[2]);
		descr_rad_ = stof(argv_g[3]);
		cg_size_ = stof(argv_g[4]);
		cg_thresh_ = stof(argv_g[5]);
		hv_clutter_reg_ = stof(argv_g[6]);
		hv_inlier_th_ = stof(argv_g[7]);
		hv_occlusion_th_ = stof(argv_g[8]);
		hv_rad_clutter_ = stof(argv_g[9]);
		hv_regularizer_ = stof(argv_g[10]);
		hv_rad_normals_ = stof(argv_g[11]);
		use_triang =  std::stoi(argv_g[12]);
		SAMPLE_POINTS_ = stof(argv_g[13]);
		distanceThrs = std::stoi(argv_g[14]);
		pointColorThrs = std::stoi(argv_g[15]);
		regionColorThrs = std::stoi(argv_g[16]);
	}else{
		model_ss_ = (float)0.001;
		scene_ss_ = (float)0.001;
		rf_rad_ = (float)0.03;
		descr_rad_ = (float)0.01;
		cg_size_ = (float)0.05;
		cg_thresh_ = (float)10.0;
		filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd";
	}
	*/

	// *************************************** Save PointCloud obtained from Kinect *************************
	/** /
	pcl::PointCloud<PointType>::Ptr save_cloud(new pcl::PointCloud<PointType>);
	pcl::PCLPointCloud2 topic_cloud;
	pcl_conversions::toPCL(*MScloud, topic_cloud);
	pcl::fromPCLPointCloud2(topic_cloud, *save_cloud);

	if(save_cloud->width == 0 || save_cloud->height == 0){
		ROS_INFO( "Cloud Empty... Return!");
		return;
	}else{
		pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_kinect_not_filtered2.pcd", *save_cloud);
		ROS_INFO( "Cloud OK... Continue!");
	}
	/**/

	// *************************************** Trasformation from camera_link frame to world frame of MScloud *************************
	/**/
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	sensor_msgs::PointCloud2 cloud_world, output, output2;
	
    //ros::Rate rate(30.0);
	ros::Duration timeout(400.0);
	try {	//camera_rgb_optical_frame
		transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time::now(), timeout);
		//transformStamped.header.frame_id = "world";
		tf2::doTransform(*MScloud, cloud_world, transformStamped);

	} catch (tf2::TransformException &ex) {
		ROS_INFO("Error Trasformation...%s",ex.what());
	}
	ROS_INFO("cloud_world size : %d", cloud_world.height * cloud_world.row_step);
	if(cloud_world.height * cloud_world.row_step == 0){
		ROS_INFO("TF_OLD_DATA ignoring data... waiting next frame!");
		return;
	}
	/**/

	// *************************************** Put Trasformed_PointCloud on PCL::PointCloud *************************
	pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_segment2 (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
	std::vector<pcl::PointCloud<PointType>::Ptr> cloud_extracted;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_triang (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<PointType>::Ptr cloud_final (new pcl::PointCloud<PointType>);

	pcl::PCLPointCloud2 cloud_pcl_world;
	pcl_conversions::toPCL(cloud_world, cloud_pcl_world);
    pcl::fromPCLPointCloud2(cloud_pcl_world, *temp_cloud);

	ROS_INFO("temp_cloud size : %d", temp_cloud->size());
	/**/
		// -----Open 3D viewer and add point cloud-----
		pcl::visualization::PCLVisualizer viewer1 ("Cloud Filtered and Extracted");
		viewer1.addCoordinateSystem(0.3);
		viewer1.addPointCloud (temp_cloud, "cloud_filtered");
		
		while (!viewer1.wasStopped ()){
			viewer1.spinOnce ();
		}
		/**/
	// *************************************** Filter on box X,Y *************************
	/**/
	// Create the filtering object in X
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;
	pass.setInputCloud (temp_cloud);
	pass.setFilterFieldName ("x");
	//pass.setFilterLimits (-0.6, +0.5); 					// SIMULATED
	pass.setFilterLimits (-0.55, +0.5);						// REAL
	//pass.setFilterLimits (-0.4, 0.7);						// REAL_SIMULATED
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_segment);

	// Create the filtering object in Y
	pass2.setInputCloud (cloud_segment);
	pass2.setFilterFieldName ("y");
	//pass2.setFilterLimits (-1.2, +1.2);						// SIMULATED - CONTROLLA BENE QUESTA MISURAù
	pass2.setFilterLimits (-0.97, -0.2);						// REAL
	//pass2.setFilterLimits (-1.42, -0.7);						// REAL_SIMULATED
	//pass.setFilterLimitsNegative (true);
	pass2.filter (*cloud_segment2);
	/**/

	// *************************************** Filter Using Segmentation  *************************
	if(cloud_segment2->size() != 0){
		/** /
		// -----Open 3D viewer and add point cloud-----
		pcl::visualization::PCLVisualizer viewer1 ("Cloud Filtered and Extracted");
		viewer1.addCoordinateSystem(0.3);
		viewer1.addPointCloud (cloud_segment2, "cloud_filtered");
		
		while (!viewer1.wasStopped ()){
			viewer1.spinOnce ();
		}
		/**/
		cloud_filtered = Plan_segmentation(cloud_segment2);

		z_plane = findZPlane(cloud_filtered);					// REAL works because good clear of isolated points

		cloud_extracted = Euclidean_cluster_extraction(cloud_filtered);
			

	}else{
		ROS_INFO("Cloud empty... cannot segmentation...");
		return;
	}

	if(use_triang){
		cloud_triang = FastTriangulation(cloud_filtered);

		pcl::copyPointCloud(*cloud_triang, *cloud_final);

		/*
		pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_filtered.pcd", *cloud_final);

		// Convert to ROS data type
		pcl::toROSMsg(*cloud_final, output);
		output.header.frame_id = "world";
		pub.publish(output);
		*/

		//Correspondence_Grouping(cloud_final,filename);
	
	}else{
		/*
		pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/scene_filtered.pcd", *cloud_filtered);

		// Convert to ROS data type
		pcl::toROSMsg(*cloud_filtered, output);
		output.header.frame_id = "/world";
		pub.publish(output);
		*/
		pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
		viewer.removeAllShapes();
		viewer.removeAllPointClouds();
		viewer.setBackgroundColor (1, 1, 1);
		viewer.addPointCloud (cloud_segment2, "scene_cloud");
		
		std::vector<std::thread> threads;
		for(int i = 0; i < cloud_extracted.size(); i++){
			int color = 5; 	//default condition
			std::string filename;
			color = ChooseColorMesh(cloud_extracted[i],i);
			switch (color){
				case (0):		// red
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
				break;
				case (1):		// green
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
				break;
				case (2):		// blue
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
				break;
				case (3):		// yellow
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
				break;
				case (4):		// try all mesh
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/cube_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/triangle_centered_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
					filename = "/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/hexagon_2k.pcd";
					threads.push_back(std::thread (Correspondence_Grouping,cloud_extracted[i], filename, i));
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
		std::vector<int> iToRemove;
		for(int i = 0; i < cloud_extracted.size(); i++){
			std::vector<int> temp_index;
			std::vector<int>::iterator iter = i_instances_global.begin();
			while ((iter = std::find_if(iter, i_instances_global.end(), CompareIds(i))) != i_instances_global.end()){
				temp_index.push_back(std::distance(i_instances_global.begin(), iter));
				iter++;
			}
			

			if(temp_index.size() == 2){
				ROS_INFO("Try to remove with red/blue of model : %d", i); 

				std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
				registered_instances.push_back(registered_instances_global[temp_index[0]]);
				registered_instances.push_back(registered_instances_global[temp_index[1]]);

				/**
				 * Hypothesis Verification
				 */
				ROS_INFO( "--- Hypotheses Verification for model ---");
				std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

				pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

				GoHv.setSceneCloud (cloud_extracted[i]);  // Scene Cloud
				GoHv.addModels (registered_instances, true);  //Models to verify
				GoHv.setResolution (0.003f); // resolution
				//GoHv.setResolutionOccupancyGrid (hv_occupancy_grid_resolution_);
				GoHv.setInlierThreshold (0.25f);
				GoHv.setOcclusionThreshold (0.01f);
				GoHv.setRegularizer (3.0);
				GoHv.setRadiusClutter (0.03f);
				GoHv.setClutterRegularizer (5.0f);
				GoHv.setDetectClutter (hv_detect_clutter_);
				GoHv.setRadiusNormals (0.05f);

				GoHv.verify ();
				GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
				bool findMod = false;

				for (int i = 0; i < hypotheses_mask.size (); i++){
					if (hypotheses_mask[i] && !findMod){
						findMod = true;
						ROS_INFO( "Instance %d is GOOD! <---", i );
						if(i == 0){
							iToRemove.push_back(temp_index[1]);	
						}else{
							iToRemove.push_back(temp_index[0]);	
						}
					}
					else{
						ROS_INFO( "Instance %d is bad!", i );
					}
				}
				if(!findMod){
					iToRemove.push_back(temp_index[rand() % 2]);
				}
				ROS_INFO( "-------------------------------" );
				
				
				/** /
				if(nCorrespondence(cloud_extracted[i], cloud_rotoTrans1) >
					nCorrespondence(cloud_extracted[i], cloud_rotoTrans2)){
					iToRemove.push_back(temp_index[1]);	
				}else{
					iToRemove.push_back(temp_index[0]);	
				}
				/**/
				ROS_INFO("Remove 1 model - numberPointCloud to remove : %d", iToRemove.size());
				
			}else if(temp_index.size() == 3){
				ROS_INFO("Try to remove with all possibility of model : %d", i);
				
				std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
				registered_instances.push_back(registered_instances_global[temp_index[0]]);
				registered_instances.push_back(registered_instances_global[temp_index[1]]);
				registered_instances.push_back(registered_instances_global[temp_index[2]]);

				/**
				 * Hypothesis Verification
				 */
				ROS_INFO( "--- Hypotheses Verification ---");
				std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

				pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

				GoHv.setSceneCloud (cloud_extracted[i]);  // Scene Cloud
				GoHv.addModels (registered_instances, true);  //Models to verify
				GoHv.setResolution (0.003f); // resolution
				//GoHv.setResolutionOccupancyGrid (hv_occupancy_grid_resolution_);
				GoHv.setInlierThreshold (0.25f);
				GoHv.setOcclusionThreshold (0.01f);
				GoHv.setRegularizer (3.0);
				GoHv.setRadiusClutter (0.03f);
				GoHv.setClutterRegularizer (5.0f);
				GoHv.setDetectClutter (hv_detect_clutter_);
				GoHv.setRadiusNormals (0.05f);

				GoHv.verify ();
				GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
				bool findMod = false;

				for (int i = 0; i < hypotheses_mask.size (); i++){
					if (hypotheses_mask[i]){
						findMod = true;
						ROS_INFO( "Instance %d is GOOD! <---", i );
						if(i == 0){
							iToRemove.push_back(temp_index[1]);	
							iToRemove.push_back(temp_index[2]);	
						}else if(i == 1){
							iToRemove.push_back(temp_index[0]);	
							iToRemove.push_back(temp_index[2]);	
						}else{
							iToRemove.push_back(temp_index[0]);	
							iToRemove.push_back(temp_index[1]);	
						}
					}
					else{
						ROS_INFO( "Instance %d is bad!", i );
					}
				}
				if(!findMod){
					int r_temp = rand() % 3;
					iToRemove.push_back(temp_index[r_temp]);
					iToRemove.push_back(temp_index[(r_temp + 1) % 3]);
				}
				ROS_INFO( "-------------------------------" );
				/*
				std::vector<int> nCorr_vec = {nCorrespondence(cloud_extracted[i],cloud_rotoTrans1),
						 						nCorrespondence(cloud_extracted[i], cloud_rotoTrans2),
						 						nCorrespondence(cloud_extracted[i], cloud_rotoTrans3)};
				int maxCorr = std::distance(nCorr_vec.begin(), std::max_element(nCorr_vec.begin(), nCorr_vec.end()));
				if(maxCorr == 0){
					iToRemove.push_back(temp_index[1]);
					iToRemove.push_back(temp_index[2]);
				}else if(maxCorr == 1){
					iToRemove.push_back(temp_index[0]);
					iToRemove.push_back(temp_index[2]);
				}else{
					iToRemove.push_back(temp_index[0]);
					iToRemove.push_back(temp_index[1]);
				}
				*/
				ROS_INFO("Remove 2 model - numberPointCloud to remove : %d", iToRemove.size());
			}else{
				ROS_INFO("Found only 1 meshes of model : %d", i);
			}
		}

		ROS_INFO("registered_instances_global size %d", registered_instances_global.size());
		for (std::size_t i = 0; i < registered_instances_global.size (); ++i){
			if(std::find(iToRemove.begin(), iToRemove.end(), i) == iToRemove.end()){
				//pcl::io::savePCDFileASCII ("/home/michele/catkin_ws/src/metapackages/homework1_test/meshes/" + std::to_string(i) + "cloud_found.pcd", *registered_instances_global[i]);
				ROS_INFO("Pointcloud added : %d", i);

				// RotoTranslate models found
				pcl::PointCloud<PointType>::Ptr cloud_rotoTrans (new pcl::PointCloud<PointType>);
				//int cl = ChooseColorMesh(registered_instances_global[i],i);
				//if(cl == 3){
				//	*cloud_rotoTrans = *registered_instances_global[i];		// Yellow not traslated
				//}else{
					cloud_rotoTrans = rotoTraslatePCD(registered_instances_global[i]);
				//}

				CloudStyle clusterStyle = style_black;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (cloud_rotoTrans, clusterStyle.r, clusterStyle.g, clusterStyle.b);
				viewer.addPointCloud (cloud_rotoTrans, instance_color_handler,  std::to_string(i));
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, std::to_string(i));
			}
		}
		while (!viewer.wasStopped ()){
			viewer.spinOnce ();
		}

	}
}


int main(int argc, char **argv){	
	argc_g = argc;
	for (int i = 1; i < argc; i++){
        argv_g[i-1] = argv[i];
    }

	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ROS_INFO("---------- Parameters list already set ----------");
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
	ROS_INFO("     sample_point_triangulation");
	ROS_INFO("     region_growing_setDistanceThreshold");
	ROS_INFO("     region_growing_setPointColorThreshold");
	ROS_INFO("     region_growing_setRegionColorThreshold");
	ROS_INFO("     angular_resolution");
	ROS_INFO("     setUnseenToMaxRange");
	ROS_INFO("     use_triangulation");
	ROS_INFO("     SAMPLE_POINTS_");
	ROS_INFO("-------------------------------------------");

	
	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	pub2 = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud_inliers", 1);


	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, Pcl_camera);
	//ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/hd/points", 1, Pcl_camera);	
	//ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect_cloud", 1, Pcl_camera);	
	
	ros::spin();
	

	file.close();
	file2.close();

	return 0;

}

