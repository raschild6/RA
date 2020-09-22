#include "node_a.h"

using namespace std;
using namespace ros;
using namespace pcl;
using namespace apriltag_ros;

typedef PointXYZRGBA PointType;
typedef Normal NormalType;
typedef ReferenceFrame RFType;
typedef SHOT352 DescriptorType;

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
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
									  
int distanceThrs = 10, pointColorThrs = 200, regionColorThrs = 50, minClusterSize = 200, maxClusterSize = 1500;
double minCutZ = 0, maxCutZ = 1.98; 
float z_plane = 1.0; 

float setInTh = 0.025;
float paramsGHV[6] = {0.003, 0.01, 3.0, 0.03, 5.0, 0.05};

vector<tuple<PointCloud<PointType>::ConstPtr, int>> registered_instances_tuple;
vector<tuple<PointCloud<PointType>::ConstPtr, int>> cloud_to_publish;
int meshTypeCloud [50];

int argc_g; 
string argv_g[100];
int typeRun = 2;
std_msgs::String name_viewer; 

Publisher pub, pub2;

int Ksearch = 50;
int setMaxIter1 = 1000;
int setMaxIter2 = 1000;
double setDistThr1 = 0.00001;
double setDistThr2 = 0.1;
double yRobot = 0.3, yWall = 0.05;

std_msgs::String link_tf; 
std_msgs::String nPoints; 

int SAMPLE_POINTS_ = 10000;

// POSSIBLE TRIANGLE : 0.001, 0.001, 0.03, 0.03, 0.01, 10.0, 5.0, 0.18, 0.01, 0.03, 3.0, 0.05 		
// POSSIBLE CUBE 	 : 0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.01, 0.01, 0.03, 3.0, 0.05 
// POSSIBLE HEXAGON  : 0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.30, 0.01, 0.03, 3.0, 0.05

float paramsHex[12] = 	{0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.01, 0.01, 0.03, 3.0, 0.05}, 
	paramsCube[12] = 	{0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.01, 0.01, 0.03, 3.0, 0.05}, 
	paramsTrian[12] = 	{0.001, 0.001, 0.03, 0.03, 0.03, 10.0, 5.0, 0.005, 0.01, 0.03, 3.0, 0.05};


bool stampResult = false;


// ----- FAST TRIANGULATION ----- */
/*
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

*/

void publishResults(){
	AprilTagDetectionArray tag_detection_array;

	for(tuple<PointCloud<PointType>::ConstPtr, int> t : cloud_to_publish){
		
		PointCloud<PointType>::Ptr cloud (new PointCloud<PointType>);	
		copyPointCloud(*get<0>(t), *cloud);

		Eigen::Vector4f centroid;

        compute3DCentroid (*cloud, centroid);
		ROS_INFO("centroid(position) [x,y,z] = [%f,%f,%f]", centroid[0], centroid[1], centroid[2]);

		//geometry_msgs::Quaternion orientation = mRot2Quat(rotation_matrix);		// non ho la matrice di rotazione
		
		geometry_msgs::PoseWithCovarianceStamped tag_poseCovStamp;
		geometry_msgs::PoseWithCovariance tag_poseCov;
		geometry_msgs::Pose tag_pose;
		
		tag_pose.position.x = centroid[0];
		tag_pose.position.y = centroid[1];
		tag_pose.position.z = centroid[2];
		tf2::Quaternion quat_tf; 
		quat_tf.setRPY(0.0, 0.0, 0.0);
		geometry_msgs::Quaternion quat_msg;
   		tf2::convert(quat_msg, quat_tf); 
		tag_pose.orientation = quat_msg; 
		tag_poseCovStamp.header.frame_id = "camera_link";
		ROS_INFO("Quaternion [x,y,z,w] = [%f,%f,%f,%f]", tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w);
		/*
		ROS_INFO("eigen_vectors coeff:");
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				ROS_INFO("coeff (%d,%d): %f", i,j,eigen_vectors.coeff(i,j));
		ROS_INFO("eigen_values coeff:");
		float maxEigenValue = eigen_values.coeff(0);
		ROS_INFO("coeff (0): %f", i,eigen_values.coeff(0));
		int maxId_eigenvalue = 0;
		for(int i = 1; i < 3; i++){
			if(eigen_values.coeff(i) > maxEigenValue){
				maxEigenValue = eigen_values.coeff(i);
				maxId_eigenvalue = i;
			}
			ROS_INFO("coeff (%d): %f", i,eigen_values.coeff(i));
		}
		
		int secondEigenValue = maxEigenValue == 0 ? 1 : maxEigenValue == 1 ? 2 : maxEigenValue == 2 ? 0 : -1,
			thirdEigenValue = -1;   
		do{
			int temp = rand() % 3;
			if(maxEigenValue != temp && secondEigenValue != temp)
				thirdEigenValue = temp;
		}while(thirdEigenValue == -1);
		*/	

		AprilTagDetection tag_detection;
		tag_poseCov.pose = tag_pose;
		tag_poseCovStamp.pose = tag_poseCov;
		tag_detection.pose = tag_poseCovStamp;
		tag_detection.id = {get<1>(t)};
		tag_detection_array.detections.push_back(tag_detection);
	}

	pub2.publish(tag_detection_array);
	spinOnce();
}

void normalsVis (PointCloud<PointType>::ConstPtr cloud, PointCloud<Normal>::ConstPtr normals){
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	if(stampResult){
		visualization::PCLVisualizer viewer (name_viewer.data.c_str());
		viewer.setBackgroundColor (255, 255, 255);
		visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
		viewer.addPointCloud<PointType> (cloud, rgb, "sample cloud");
		viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		viewer.addPointCloudNormals<PointType, Normal> (cloud, normals, 10, 0.05, "normals");
		viewer.addCoordinateSystem (1.0);
		viewer.initCameraParameters ();
		while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	}
}

void simpleVis (PointCloud<PointType>::ConstPtr cloud){
  	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	if(stampResult){
		visualization::PCLVisualizer viewer (name_viewer.data.c_str());
		viewer.setBackgroundColor (255, 255, 255);
		viewer.addPointCloud<PointType> (cloud, "sample cloud");
		viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		viewer.addCoordinateSystem (1.0);
		viewer.initCameraParameters ();
		while (!viewer.wasStopped ()){
			viewer.spinOnce ();
		}
		}
}

PointCloud<PointType>::Ptr rotoTraslatePCD(const PointCloud<PointType>::ConstPtr &cloud){
	PointType minPt, maxPt;
  	getMinMax3D (*cloud, minPt, maxPt);
	
	Eigen::Affine3f trasform = Eigen::Affine3f::Identity();
	if(minPt.z > z_plane){
		trasform.translation() << 0.0, 0.0, (z_plane - minPt.z); 
	}else{
		trasform.translation() << 0.0, 0.0, (z_plane - minPt.z);
	}
	
	//trasform.rotate(Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitZ()));
	trasform.matrix();

	PointCloud<PointType>::Ptr temp_cloud (new PointCloud<PointType>());
	transformPointCloud(*cloud, *temp_cloud, trasform);

	return temp_cloud;			
}

float findZPlane(const PointCloud<PointType>::ConstPtr &cloud){
	PointType minPt, maxPt;
  	getMinMax3D (*cloud, minPt, maxPt);
	ROS_INFO("Minimum Z Point of Plan Segmentation : %f", minPt.z);
	return minPt.z;
}

int ChooseColorMesh(const PointCloud<PointType>::ConstPtr &cloud, int i_cloud){
	int r = 0, g = 0, b = 0;
	int size_cloud = cloud->points.size();
	
	//name_viewer.data = "Cloud Colored Number: " + to_string(i_cloud); simpleVis(cloud);

	for(int i = 0; i < cloud->points.size(); i++){
		r += cloud->points[i].r;
		g += cloud->points[i].g;
		b += cloud->points[i].b;
	}

	r = r / cloud->points.size();
	g = g / cloud->points.size();
	b = b / cloud->points.size();

	ROS_INFO("%d cloud, rgb color : (%d, %d, %d) ", i_cloud, r,g,b);

	if(r < 80 && g < 80 && b < 80){			// discard (table or noise)
		ROS_INFO("%d ........... Discarded ...........",i_cloud);
		return 5; 			
	}else if(r >= 80 && g >= 80 && b <= 80){
		ROS_INFO("%d ........... Yellow ...........",i_cloud);
		return 3;
	}else if(r >= 80 && g <= 80 && b <= 80){
		ROS_INFO("%d ........... Red ...........",i_cloud);
		return 0;
	}else if(r <= 80 && g >= 80 && b <= 80){
		ROS_INFO("%d ........... Green ...........",i_cloud);
		return 1;
	}else if(r <= 80 && g <= 80 && b >= 80){
		ROS_INFO("%d ........... Blue ...........",i_cloud);
		return 2;
	}else{
		ROS_INFO("%d ........... Probably colors missed! Wait next frame ...........",i_cloud);
		return 5;							
	}
	// REAL color
	/** 
	for(int i = 0; i < cloud->points.size(); i++){
		vector<int> rgb_temp_vector = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
		vector<int> rgb_vector = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
		sort(rgb_vector.begin(),rgb_vector.end(), greater<int>());
		if((rgb_vector[0] - rgb_vector[1] > 35) && (rgb_vector[0] - rgb_vector[2] > 35)){

			vector<int>::iterator it = find(rgb_temp_vector.begin(), rgb_temp_vector.end(), rgb_vector[0]);
			int index = distance(rgb_temp_vector.begin(), it);
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
	
	vector<int> rgb_temp_vector = {r, g, b};
	vector<int> rgb_vector = {r, g, b};
	sort(rgb_vector.begin(),rgb_vector.end(), greater<int>());
	if((rgb_vector[0] - rgb_vector[1] > 500000) && (rgb_vector[0] - rgb_vector[2] > 500000)){								// find red or green

		vector<int>::iterator it = find(rgb_temp_vector.begin(), rgb_temp_vector.end(), rgb_vector[0]);
		int index = distance(rgb_temp_vector.begin(), it);
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
	/**/
}

double computeCloudResolution (const PointCloud<PointType>::ConstPtr &cloud){
	double res = 0.0;
	int n_points = 0;
	int nres;
	vector<int> indices (2);
	vector<float> sqr_distances (2);
	search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i){
		if (! isfinite ((*cloud)[i].x)){
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

void Correspondence_Grouping (const PointCloud<PointType>::ConstPtr &sceneConst, string filename, int i_cloud, float params[]){
	ofstream result;
	size_t posI = filename.find("meshes/");
	string sTemp = filename.substr(posI+7);
	string nameMesh = sTemp.substr(0, sTemp.length()-4);
	auto end = chrono::system_clock::now();
    time_t end_time = chrono::system_clock::to_time_t(end);
	string datetime = ctime(&end_time);
	replace(datetime.begin(), datetime.end(), ' ', '_');
	string namefile = "/home/michele/catkin_ws/src/homework1_test/test/CG_" + to_string(i_cloud) + "_" + nameMesh.c_str() + "_" + datetime + ".txt"; 
	result.open (namefile);
	result << " ---- " << filename << " ---- " << "\n\n";

	PointCloud<PointType>::Ptr scene (new PointCloud<PointType> ());
	PointCloud<PointType>::Ptr model (new PointCloud<PointType> ());

	*scene = *sceneConst;

	if (io::loadPCDFile<PointType> (filename, *model) == -1){
    	PCL_ERROR("Couldn't read .pcd file model");
  	}

	PointCloud<PointType>::Ptr model_keypoints (new PointCloud<PointType> ());
	PointCloud<PointType>::Ptr scene_keypoints (new PointCloud<PointType> ());
	PointCloud<NormalType>::Ptr model_normals (new PointCloud<NormalType> ());
	PointCloud<NormalType>::Ptr scene_normals (new PointCloud<NormalType> ());
	PointCloud<DescriptorType>::Ptr model_descriptors (new PointCloud<DescriptorType> ());
	PointCloud<DescriptorType>::Ptr scene_descriptors (new PointCloud<DescriptorType> ());

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
	result << "Normals radius:                 " << hv_rad_normals_ << "\n";

	//
	//  Compute Normals
	//
	NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample Clouds to Extract keypoints
	//
	UniformSampling<PointType> uniform_sampling;
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
	SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	CorrespondencesPtr model_scene_corrs (new Correspondences ());
	KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);
	vector<int> model_good_keypoints_indices;
	vector<int> scene_good_keypoints_indices;

	for (size_t i = 0; i < scene_descriptors->size (); ++i){
		vector<int> neigh_indices (1);
		vector<float> neigh_sqr_dists (1);
		if (!isfinite (scene_descriptors->at (i).descriptor[0])){  //skipping NaNs
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.45f){
			Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
			model_good_keypoints_indices.push_back (corr.index_query);
			scene_good_keypoints_indices.push_back (corr.index_match);
		}
	}
	result << "model_good_keypoints_indices selected: " << model_good_keypoints_indices.size() << "\n";
	result << "scene_good_keypoints_indices selected: " << scene_good_keypoints_indices.size() << "\n";

	PointCloud<PointType>::Ptr model_good_kp (new PointCloud<PointType> ());
	PointCloud<PointType>::Ptr scene_good_kp (new PointCloud<PointType> ());
	copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);
	result << "Correspondences found: " << model_scene_corrs->size () << "\n";

	//
	//  Actual Clustering
	//
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	vector<Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_){
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		PointCloud<RFType>::Ptr model_rf (new PointCloud<RFType> ());
		PointCloud<RFType>::Ptr scene_rf (new PointCloud<RFType> ());

		BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
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
		Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
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
		GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
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
  	vector<PointCloud<PointType>::ConstPtr> instances;

	for (size_t i = 0; i < rototranslations.size (); ++i){
		PointCloud<PointType>::Ptr rotated_model (new PointCloud<PointType> ());
		transformPointCloud (*model, *rotated_model, rototranslations[i]);
		instances.push_back (rotated_model);
	}

	/**
	 * ICP
	 */
	vector<PointCloud<PointType>::ConstPtr> registered_instances;
	result << "--------- ICP ---------" << "\n";
	for (size_t i = 0; i < rototranslations.size (); ++i){
		IterativeClosestPoint<PointType, PointType> icp;
		icp.setMaximumIterations (icp_max_iter_);
		icp.setMaxCorrespondenceDistance (icp_corr_distance_);
		icp.setInputTarget (scene);
		icp.setInputSource (instances[i]);
		PointCloud<PointType>::Ptr registered (new PointCloud<PointType>);
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
	vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

	GlobalHypothesesVerification<PointType, PointType> GoHv;
	
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
	for (size_t i = 0; i < registered_instances.size (); ++i){
		if (hypotheses_mask.size() != 0 && hypotheses_mask[i] ){
			findOne = true;
			tuple<PointCloud<PointType>::ConstPtr, int> temp_tuple;
			temp_tuple = make_tuple(registered_instances[i], i_cloud);
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

vector<PointCloud<PointType>::Ptr> Euclidean_cluster_extraction(const PointCloud<PointType>::ConstPtr &cloud_filtered){

  	PointCloud <PointXYZRGB>::Ptr temp1 (new PointCloud<PointXYZRGB>);
	copyPointCloud(*cloud_filtered, *temp1);

	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
	tree->setInputCloud (temp1);

	vector<PointIndices> cluster_indices;
	RegionGrowingRGB<PointXYZRGB> reg;
	reg.setInputCloud (temp1);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (distanceThrs);			//25
	reg.setPointColorThreshold (pointColorThrs);		//27 - default 1225
	reg.setRegionColorThreshold (regionColorThrs);		//27 - default 10
	reg.setMinClusterSize (minClusterSize);
	reg.setMaxClusterSize (maxClusterSize);
	reg.extract (cluster_indices);

  	PointCloud <PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	PointCloud <PointType>::Ptr color_cloud (new PointCloud<PointType>);
	copyPointCloud(*colored_cloud, *color_cloud);
	
	//name_viewer.data = "Cloud Colored"; simpleVis(color_cloud);

	vector<PointCloud<PointType>::Ptr> models_cloud_vector;

	int j = 0;
	for ( vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		PointCloud<PointType>::Ptr cloud_temp (new PointCloud<PointType>);

		ROS_INFO("cloud_vector size : %d, indices number : %d", models_cloud_vector.size(), it->indices.size());

		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
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

vector<PointCloud<PointType>::Ptr> remove_plane(const PointCloud<PointType>::ConstPtr &cloud, 
					SACSegmentation<PointType> seg, SACSegmentationFromNormals<PointType, Normal> seg2,
					ExtractIndices<PointType> extract, ExtractIndices<PointType> extract2, 
					NormalEstimation<PointType, Normal> ne){
	
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointIndices::Ptr inliers (new PointIndices);

	PointCloud<PointType>::Ptr cloud_temp (new PointCloud<PointType>);
	PointCloud<PointType>::Ptr cloud_outliers (new PointCloud<PointType>);	
	PointCloud<PointType>::Ptr cloud_inliers (new PointCloud<PointType>);
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

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

PointCloud<PointType>::Ptr Plan_segmentation(const PointCloud<PointType>::ConstPtr &cloud){
	PointCloud<PointType>::Ptr cloud_outliers (new PointCloud<PointType>);	
	PointCloud<PointType>::Ptr cloud_outliers_down (new PointCloud<PointType>);	
	PointCloud<PointType>::Ptr cloud_inliers (new PointCloud<PointType>);
	PointCloud<PointType>::Ptr cloud_outliers_filtered (new PointCloud<PointType>);
	vector<PointCloud<PointType>::Ptr> out_in_cloud;

	// Create the segmentation object
	SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMaxIterations(setMaxIter1);
	seg.setMethodType (SAC_RANSAC);
	seg.setDistanceThreshold (setDistThr1);
	
	ExtractIndices<PointType> extract;
	ExtractIndices<PointType> extract2;
	extract.setNegative (true);			
	
	// Estimate point normals
	NormalEstimation<PointType, Normal> ne;
	search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType> ());
	ne.setSearchMethod (tree);
	ne.setKSearch (Ksearch);

	SACSegmentationFromNormals<PointType, Normal> seg2; 
	seg2.setModelType (SACMODEL_PERPENDICULAR_PLANE);
	seg2.setMaxIterations(setMaxIter2);
	seg2.setMethodType (SAC_RANSAC);
	seg2.setDistanceThreshold (setDistThr2);
	seg2.setDistanceFromOrigin (1.99);

	copyPointCloud(*cloud, *cloud_outliers);		// use if you want to run next code in loop

	//Point in Plane : x = -0.22833 ; y = -0.546301 ; z = 1.99363;

	// Create the filtering object in Z
	pcl::PassThrough<PointType> pass3;
	pcl::PassThrough<PointType> pass4;
	// ---------MIN Z----------:         1.983194
	// ---------MAX Z----------:         1.997834
	pass3.setInputCloud (cloud_outliers);
	pass3.setFilterFieldName ("z");	
	pass3.setFilterLimits (maxCutZ, maxCutZ + 0.3);
	pass3.filter (*cloud_outliers_down);
	
	pass4.setInputCloud (cloud_outliers);
	pass4.setFilterFieldName ("z");	
	pass4.setFilterLimits (minCutZ, maxCutZ);
	pass4.filter (*cloud_outliers);

	name_viewer.data = "Cloud Outliers Down"; simpleVis(cloud_outliers_down);
	//name_viewer.data = "Cloud Outliers Up"; simpleVis(cloud_outliers);
	
	out_in_cloud = remove_plane(cloud_outliers_down, seg, seg2, extract, extract2, ne);
	if(out_in_cloud.size() > 1){
		//cloud_outliers = out_in_cloud[0];
		cloud_inliers = out_in_cloud[1];
	}else{
		return cloud_outliers_filtered;
	}

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud (cloud_inliers);
	sor.setMeanK (20);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_inliers);

	name_viewer.data = "Cloud Inliers Filtered"; simpleVis(cloud_inliers);
	//name_viewer.data = "Cloud Outliers"; simpleVis(cloud_outliers);

	PointType minPt, maxPt;
  	getMinMax3D (*cloud_inliers, minPt, maxPt);
	
	ROS_INFO("---------MIN X----------:         %f", minPt.x);
	ROS_INFO("---------MAX X----------:         %f", maxPt.x);
	ROS_INFO("---------MIN Y----------:         %f", minPt.y);
	ROS_INFO("---------MAX Y----------:         %f", maxPt.y);
	ROS_INFO("---------MIN Z----------:         %f", minPt.z);
	ROS_INFO("---------MAX Z----------:         %f", maxPt.z);
	
	pcl::PassThrough<PointType> pass;
	pcl::PassThrough<PointType> pass2;

	// Create the filtering object in X
	// ---------MIN X----------:         -0.553821
	// ---------MAX X----------:         0.467484
	pass.setInputCloud (cloud_outliers);	
	pass.setFilterFieldName ("x");	
	pass.setFilterLimits (-0.6, 0.6);
	pass.filter (*cloud_outliers);

	pass.setInputCloud (cloud_outliers);
	pass.setFilterFieldName ("x");	
	pass.setFilterLimits (minPt.x, maxPt.x);
	pass.filter (*cloud_outliers);

	// Create the filtering object in Y
	// ---------MIN Y----------:         -0.806813
	// ---------MAX Y----------:         0.347452
	pass2.setInputCloud (cloud_outliers);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (-0.9, 0.5);
	pass2.filter (*cloud_outliers);

	pass2.setInputCloud (cloud_outliers);
	pass2.setFilterFieldName ("y");
	pass2.setFilterLimits (minPt.y + yRobot, maxPt.y - yWall);
	pass2.filter (*cloud_outliers);

	// Create the filtering object
  	StatisticalOutlierRemoval<PointType> sor2;
  	sor2.setInputCloud (cloud_outliers);
  	sor2.setMeanK (50);
  	sor2.setStddevMulThresh (1.2);
	sor2.filter (*cloud_outliers_filtered);
	

	name_viewer.data = "Cloud Outliers Filtered"; simpleVis(cloud_outliers_filtered);

	return cloud_outliers_filtered;
}

void Pcl_camera(const sensor_msgs::PointCloud2ConstPtr& MScloud){
	

	// *************************************** Trasformation from camera_link frame to world frame of MScloud *************************
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	sensor_msgs::PointCloud2 cloud_world, output, output2;

    //Rate rate(30.0);
	Duration timeout(400.0);
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

	// *************************************** Put Trasformed_PointCloud on PointCloud *************************
	PointCloud<PointType>::Ptr temp_cloud(new PointCloud<PointType>);
	PointCloud<PointType>::Ptr cloud_segment (new PointCloud<PointType>);
	PointCloud<PointType>::Ptr cloud_segment2 (new PointCloud<PointType>);
	PointCloud<PointType>::Ptr cloud_filtered (new PointCloud<PointType>);
	vector<PointCloud<PointType>::Ptr> cloud_extracted;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_triang (new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointType>::Ptr cloud_final (new PointCloud<PointType>);

	
	//PCLPointCloud2 cloud_pcl_world;
	//pcl_conversions::toPCL(cloud_world, cloud_pcl_world);
    //fromPCLPointCloud2(cloud_pcl_world, *temp_cloud);
	//fromROSMsg(*MScloud.get(), *temp_cloud.get());
	//fromROSMsg(*cloud_world, *temp_cloud);
	//toROSMsg(*temp_cloud, output);
	//pub.publish(output);

	
	PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_world,pcl_pc2);
    fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	ROS_INFO("temp_cloud size : %d", temp_cloud->size());
	name_viewer.data = "Cloud Temp"; 
	simpleVis(temp_cloud);

	// *************************************** Filter Using Segmentation  *************************
	  
	if(temp_cloud->size() != 0){
		
		
		cloud_filtered = Plan_segmentation(temp_cloud);
	
		if(cloud_filtered->size() == 0){
			ROS_ERROR("----- Plan segmentation fail! return -----");
			return;
		}
		ROS_INFO("----- Plan segmentation done! -----");

		// if(typeRun == 0) z_plane = findZPlane(cloud_filtered);					// REAL works because good clear of isolated points

		cloud_extracted = Euclidean_cluster_extraction(cloud_filtered);
		
		ROS_INFO("----- Region Growing RGB done! -----");

	}else{
		ROS_INFO("Cloud empty... cannot segmentation...");
		return;
	}
	

	/* ------- print final result ------- * /
	pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
	viewer.setBackgroundColor (1, 1, 1);
	/**/
	
	vector<thread> threads;
	for(int i = 0; i < cloud_extracted.size(); i++){
		int color = 5; 	//default condition
		string filename;
		color = ChooseColorMesh(cloud_extracted[i],i);
		switch (color){
			case (0):		// red
				if(typeRun == 2){

					/* For using FastTriangulation
					cloud_triang = FastTriangulation(cloud_extracted[i]);
					copyPointCloud(*cloud_triang, *cloud_final);
					threads.push_back(thread (Correspondence_Grouping, cloud_final, filename, i, paramsTrian));
					*/
					
					PointType minPt, maxPt;
					getMinMax3D (*cloud_extracted[i], minPt, maxPt);
					if(minPt.z >= 1.9){
						
						meshTypeCloud[i] = 4;
						
						filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
						threads.push_back(thread (Correspondence_Grouping, cloud_extracted[i], filename, i, paramsTrian));
					}else{
						/* Remove useless points for the cube */
						pcl::PassThrough<PointType> pass;
						pass.setInputCloud (cloud_extracted[i]);	
						pass.setFilterFieldName ("z");	
						pass.setFilterLimits (minPt.z, minPt.z + 0.1);
						pass.filter (*cloud_extracted[i]);
						
						meshTypeCloud[i] = 0;

						filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
						threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
						}
				}else{
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}
			break;
			case (1):		// green
				if(typeRun == 2){
					
					meshTypeCloud[i] = 2;

					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping, cloud_extracted[i], filename, i, paramsTrian));
				}else{
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
				}
			break;
			case (2):		// blue
				if(typeRun == 2){
					/* Remove useless points for the cube */
					PointType minPt, maxPt;
					getMinMax3D (*cloud_extracted[i], minPt, maxPt);
					pcl::PassThrough<PointType> pass;
					pass.setInputCloud (cloud_extracted[i]);	
					pass.setFilterFieldName ("z");	
					pass.setFilterLimits (minPt.z, minPt.z + 0.1);
					pass.filter (*cloud_extracted[i]);

					meshTypeCloud[i] = 3;

					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
				}else{
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
				}
			break;
			case (3):		// yellow
				if(typeRun == 2){
					// Remove useless points
					PointType minPt, maxPt;
					getMinMax3D (*cloud_extracted[i], minPt, maxPt);
					// Create the filtering object in Z
					pcl::PassThrough<PointType> pass;
					pass.setInputCloud (cloud_extracted[i]);	
					pass.setFilterFieldName ("z");	
					pass.setFilterLimits (minPt.z, minPt.z + 0.1);
					pass.filter (*cloud_extracted[i]);

					meshTypeCloud[i] = 1;

					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/hexagon_cut_20k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));
				}else{
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/hexagon_cut_20k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));	
				}
			break;
			case (4):		// try all mesh
				if(typeRun == 2){
					meshTypeCloud[i] = 5;
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/hexagon_cut_20k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));
				}else{
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/cube_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsCube));
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/triangle_centered_cut_10k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsTrian));
					filename = "/home/michele/catkin_ws/src/homework1_test/meshes/hexagon_cut_20k.pcd";
					threads.push_back(thread (Correspondence_Grouping,cloud_extracted[i], filename, i, paramsHex));	
				}
			break;
			case (5):
				continue;
			break;
			default:
				ROS_INFO("Cannot continue with Correspondence Grouping!!");
				return;
			break;
		}
	}
	if(threads.size() == 0){ 	// no object for recognition
		ROS_INFO("If no object on table: Cannot continue with Correspondence Grouping! Or PointCloud could be in greyscale: wait next iteration..");
		return;
	}
	for (thread & th : threads){
		if (th.joinable()){
			th.join();
		}
	}

	ROS_INFO(" -------------------------- Thread created : %d", threads.size());
	
	vector<PointCloud<PointType>::ConstPtr> registered_instances_global;
	vector<int> i_instances_global;
	for(tuple<PointCloud<PointType>::ConstPtr, int> t : registered_instances_tuple){
		registered_instances_global.push_back(get<0>(t));
		i_instances_global.push_back(get<1>(t));
		ROS_INFO("Indices of global pointcloud found : %d", get<1>(t));
	}
	ROS_INFO("registered_instances size : %d --- instances :%d", registered_instances_global.size(),i_instances_global.size());
	
	ROS_INFO("------- Finding multi mesh on the same model -------");
	vector<int> iToStay;
	float temp_setInTh = setInTh;
	for(int i = 0; i < cloud_extracted.size(); i++){
		vector<int> temp_index;
		vector<int> temp_index_hv;
		vector<int>::iterator iter = i_instances_global.begin();
		while ((iter = find_if(iter, i_instances_global.end(), CompareIds(i))) != i_instances_global.end()){
			temp_index.push_back(distance(i_instances_global.begin(), iter));
			iter++;
		}
		
		if(temp_index.size() > 1){
			vector<PointCloud<PointType>::ConstPtr> registered_instances;
			for(int index : temp_index){
				registered_instances.push_back(registered_instances_global[index]);
			}

			bool findMod = false;
			setInTh = temp_setInTh;
			while(!(temp_index_hv.size() == 1) && setInTh < 0.4){
				findMod = false;
				setInTh += 0.005;
				temp_index_hv.clear();
				// Hypothesis Verification
				ROS_INFO( "--- Hypotheses Verification for model --- setInlierThreshold : %f", setInTh);
				vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

				GlobalHypothesesVerification<PointType, PointType> GoHv;

				GoHv.setSceneCloud (cloud_extracted[i]);  // Scene Cloud
				GoHv.addModels (registered_instances, true);  //Models to verify
				GoHv.setResolution (paramsGHV[0]); // resolution
				GoHv.setInlierThreshold (setInTh);
				GoHv.setOcclusionThreshold (paramsGHV[1]);
				GoHv.setRegularizer (paramsGHV[2]);
				GoHv.setRadiusClutter (paramsGHV[3]);
				GoHv.setClutterRegularizer (paramsGHV[4]);
				GoHv.setDetectClutter (hv_detect_clutter_);
				GoHv.setRadiusNormals (paramsGHV[5]);

				/* Backup values parameters 
					GoHv.setResolution (0.003f); // resolution
					GoHv.setInlierThreshold (setInTh);
					GoHv.setOcclusionThreshold (0.01f);
					GoHv.setRegularizer (3.0);
					GoHv.setRadiusClutter (0.03f);
					GoHv.setClutterRegularizer (5.0f);
					GoHv.setDetectClutter (hv_detect_clutter_);
					GoHv.setRadiusNormals (0.05f);
				*/

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
	for (size_t i = 0; i < registered_instances_global.size (); ++i){
		if(find(iToStay.begin(), iToStay.end(), i) != iToStay.end()){
			ROS_INFO("Pointcloud added : %d", i);

			// RotoTranslate models found
			PointCloud<PointType>::Ptr cloud_rotoTrans (new PointCloud<PointType>);
			*cloud_rotoTrans = *registered_instances_global[i];
			//cloud_rotoTrans = rotoTraslatePCD(registered_instances_global[i]);

			cloud_to_publish.push_back({cloud_rotoTrans, meshTypeCloud[i]});

			/* ------- print final result ------- * /
			CloudStyle clusterStyle = style_red;
			visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (cloud_rotoTrans, clusterStyle.r, clusterStyle.g, clusterStyle.b);
			viewer.addPointCloud (cloud_rotoTrans, instance_color_handler,  to_string(i));
			viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, to_string(i));
			/**/
		}
	}
	
	/* ------- print final result -------* /
	CloudStyle clusterStyle2 = style_black;
	visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (temp_cloud, clusterStyle2.r, clusterStyle2.g, clusterStyle2.b);
	viewer.addPointCloud (temp_cloud, instance_color_handler,  to_string(registered_instances_global.size ()));
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle2.size, to_string(registered_instances_global.size ()));
	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
	/**/

	publishResults();


}

int main(int argc, char **argv){	
	
	argc_g = argc;
	 
	if (argc < 2){
        ROS_INFO("Usage: rosrun hw1 node_hw1_task_2 [0 = Stamp disable, otherwise = Stamp enable] frame_id_1 frame_id_2 ...");
        return 0;
    }
	if(stoi(argv[1]) == 1){
		ROS_INFO("Node launched with Stamp Enable! Close windows that will appear to continue..");
		stampResult = true;
	}
	for (int i = 2; i < argc; i++){
        argv_g[i-1] = argv[i];
    }
	link_tf.data = "camera_link";
	nPoints.data = "cut_10k";
	if(argc_g > 1){

		/* Set parames hypothesis verification for multi-mesh on same model * /
		paramsGHV[0] = atof(argv_g[0].c_str());
		setInTh 	 = atof(argv_g[1].c_str());
		paramsGHV[1] = atof(argv_g[2].c_str());
		paramsGHV[2] = atof(argv_g[3].c_str());
		paramsGHV[3] = atof(argv_g[4].c_str());
		paramsGHV[4] = atof(argv_g[5].c_str());
		paramsGHV[5] = atof(argv_g[6].c_str());
		/**/
		/* Set parames triangle * /
		paramsTrian[0] = atof(argv_g[0].c_str());
		paramsTrian[1] = atof(argv_g[1].c_str());
		paramsTrian[2] = atof(argv_g[2].c_str());
		paramsTrian[3] = atof(argv_g[3].c_str());
		paramsTrian[4] = atof(argv_g[4].c_str());
		paramsTrian[5] = atof(argv_g[5].c_str());
		paramsTrian[6] = atof(argv_g[6].c_str());
		paramsTrian[7] = atof(argv_g[7].c_str());
		paramsTrian[8] = atof(argv_g[8].c_str());
		paramsTrian[9] = atof(argv_g[9].c_str());
		paramsTrian[10] = atof(argv_g[10].c_str());
		paramsTrian[11] = atof(argv_g[11].c_str());
		/**/
		/* Set parames cube * /
		paramsCube[0] = atof(argv_g[12].c_str());
		paramsCube[1] = atof(argv_g[13].c_str());
		paramsCube[2] = atof(argv_g[14].c_str());
		paramsCube[3] = atof(argv_g[15].c_str());
		paramsCube[4] = atof(argv_g[16].c_str());
		paramsCube[5] = atof(argv_g[17].c_str());
		paramsCube[6] = atof(argv_g[18].c_str());
		paramsCube[7] = atof(argv_g[19].c_str());
		paramsCube[8] = atof(argv_g[20].c_str());
		paramsCube[9] = atof(argv_g[21].c_str());
		paramsCube[10] = atof(argv_g[22].c_str());
		paramsCube[11] = atof(argv_g[23].c_str());
		/**/
		/* Set parames hexagon * /
		paramsHex[0] = atof(argv_g[24].c_str());
		paramsHex[1] = atof(argv_g[25].c_str());
		paramsHex[2] = atof(argv_g[26].c_str());
		paramsHex[3] = atof(argv_g[27].c_str());
		paramsHex[4] = atof(argv_g[28].c_str());
		paramsHex[5] = atof(argv_g[29].c_str());
		paramsHex[6] = atof(argv_g[30].c_str());
		paramsHex[7] = atof(argv_g[31].c_str());
		paramsHex[8] = atof(argv_g[32].c_str());
		paramsHex[9] = atof(argv_g[33].c_str());
		paramsHex[10] = atof(argv_g[34].c_str());
		paramsHex[11] = atof(argv_g[35].c_str());
		/**/
	}

	init(argc, argv, "node_a"); 
	NodeHandle n;

	ROS_INFO("---------- Parameters list already set ----------");
	ROS_INFO("     typeRun:\t\t\t\t\t %d", typeRun);
	ROS_INFO("     show_keypoints_:\t\t\t\t %s", show_keypoints_ ? "true" : "false");
	ROS_INFO("     show_correspondences_:\t\t\t %s", show_correspondences_ ? "true" : "false");
	ROS_INFO("     use_cloud_resolution_:\t\t\t %s", use_cloud_resolution_ ? "true" : "false");
	ROS_INFO("     use_hough_:\t\t\t\t %s", use_hough_ ? "true" : "false");
	ROS_INFO("     hv_detect_clutter_:\t\t\t %s", hv_detect_clutter_ ? "true" : "false");
	ROS_INFO("     setUnseenToMaxRange:\t\t\t %s", setUnseenToMaxRange ? "true" : "false");
	ROS_INFO("     angular_resolution:\t\t\t %f", angular_resolution);
	ROS_INFO("     model_ss_:\t\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[0], paramsCube[0], paramsHex[0]);
	ROS_INFO("     scene_ss_:\t\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[1], paramsCube[1], paramsHex[1]);
	ROS_INFO("     rf_rad_:\t\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[2], paramsCube[2], paramsHex[2]);
	ROS_INFO("     descr_rad_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[3], paramsCube[3], paramsHex[3]);
	ROS_INFO("     cg_size_:\t\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[4], paramsCube[4], paramsHex[4]);
	ROS_INFO("     cg_thresh_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[5], paramsCube[5], paramsHex[5]);
	ROS_INFO("     hv_clutter_reg_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[6], paramsCube[6], paramsHex[6]);
	ROS_INFO("     hv_inlier_th_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[7], paramsCube[7], paramsHex[7]);
	ROS_INFO("     hv_occlusion_th_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[8], paramsCube[8], paramsHex[8]);
	ROS_INFO("     hv_rad_clutter_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[9], paramsCube[9], paramsHex[9]);
	ROS_INFO("     hv_regularizer_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[10], paramsCube[10], paramsHex[10]);
	ROS_INFO("     hv_rad_normals_:\t\t\t\t [t,c,h] -> [%.4f,%.4f,%.4f]", paramsTrian[11], paramsCube[11], paramsHex[11]);
	ROS_INFO("     region_growing_setDistanceThreshold:\t %d", distanceThrs);		// Allows to set distance threshold.
	ROS_INFO("     region_growing_setPointColorThreshold:\t %d", pointColorThrs);	// This method specifies the threshold value for color test between the points. This kind of testing is made at the first stage of the algorithm(region growing). If the difference between points color is less than threshold value, then they are considered to be in the same region.
	ROS_INFO("     region_growing_setRegionColorThreshold:\t %d", regionColorThrs);	// This method specifies the threshold value for color test between the regions. This kind of testing is made at the second stage of the algorithm(region merging). If the difference between segments color is less than threshold value, then they are merged together.
	ROS_INFO("     region_growing_setMinClusterSize:\t\t %d", minClusterSize);
	ROS_INFO("     region_growing_setManClusterSize:\t\t %d", maxClusterSize);
	ROS_INFO("     Coordinate of Link:\t\t\t %s", link_tf.data.c_str());
	ROS_INFO("     setMaxIterazions 1:\t\t\t %d", setMaxIter1);
	ROS_INFO("     setMaxIterazions 2:\t\t\t %d", setMaxIter2);
	ROS_INFO("     setDistThr 1:\t\t\t\t %f", setDistThr1);
	ROS_INFO("     setDistThr 2:\t\t\t\t %f", setDistThr2);
	ROS_INFO("     extraSpaceRobot_toRemove:\t\t\t %f", yRobot);
	ROS_INFO("     extraSpaceWall_toRemove:\t\t\t %f", yWall);
	ROS_INFO("-------------------------------------------");

	/* Helper parameters:

		ROS_INFO("     --region_growing_setDistanceThreshold:		Allows to set distance threshold.	
		ROS_INFO("     --region_growing_setPointColorThreshold:		This method specifies the threshold value for color test between the points. 
																	This kind of testing is made at the first stage of the algorithm(region growing). 
																	If the difference between points color is less than threshold value, then they are considered to be in the same region.
		ROS_INFO("     --region_growing_setRegionColorThreshold: 	This method specifies the threshold value for color test between the regions. 
																	This kind of testing is made at the second stage of the algorithm(region merging). 
																	If the difference between segments color is less than threshold value, then they are merged together.
		ROS_INFO("     --model_ss val:              				Model uniform sampling radius 
		ROS_INFO("     --scene_ss val:             			 		Scene uniform sampling radius 
		ROS_INFO("     --rf_rad val:                		 		Reference frame radius 
		ROS_INFO("     --descr_rad val:             		 		Descriptor radius 
		ROS_INFO("     --cg_size val:               		 		Cluster size 
		ROS_INFO("     --cg_thresh val:             		 		Clustering threshold 
		ROS_INFO("     --icp_max_iter val:          		 		ICP max iterations number 
		ROS_INFO("     --icp_corr_distance val:     		 		ICP correspondence distance 
		ROS_INFO("     --hv_clutter_reg val:        		 		Clutter Regularizer 
		ROS_INFO("     --hv_inlier_th val:          		 		Inlier threshold 
		ROS_INFO("     --hv_occlusion_th val:       		 		Occlusion threshold 
		ROS_INFO("     --hv_rad_clutter val:        		 		Clutter radius 
		ROS_INFO("     --hv_regularizer val:        		 		Regularizer value 
		ROS_INFO("     --hv_rad_normals val:        		 		Normals radius 
		ROS_INFO("     --hv_detect_clutter val:     		 		TRUE if clutter detect enabled
	
	
	*/


	pub = n.advertise<sensor_msgs::PointCloud2> ("/my_cloud", 1);
	pub2 = n.advertise<AprilTagDetectionArray> ("/pose_objects", 1);
	
	Subscriber sub;
	if(typeRun == 0){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/hd/points", 1, Pcl_camera);					// REAL
	}else if(typeRun == 1){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect_cloud", 1, Pcl_camera);						// REAL-SIMULATED
	}else if(typeRun == 2){
		sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, Pcl_camera);		// SIMULATED (COLORED)
	}	
	
	spin();

	return 0;

}

