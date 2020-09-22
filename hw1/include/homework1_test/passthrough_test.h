#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class Passthrough{

  public:
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    
    Passthrough();
    Passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cld);
    ~Passthrough();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtering();
};
