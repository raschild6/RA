#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <homework1_test/passthrough_test.h>


Passthrough::Passthrough(){  
}
Passthrough::Passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cld){
  cloud -> points = cld -> points;
}
Passthrough::~Passthrough(){
  delete &cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Passthrough::filtering(){
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  return cloud_filtered;
}

int main(int argc, char **argv){
  return 0;
}
