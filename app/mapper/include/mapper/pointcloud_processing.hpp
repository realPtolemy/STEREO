#include "app/shared_state.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill

#include <lz4.h>
#include <iomanip>

// class PointcloudProcessing{
// private:
//     Client client;
//     void sendPC();
// public:
//     PointcloudProcessing();
//     PointcloudProcessing(int port);

//     void find_clusters(Pointcloud& pc);

//     void serializePC(Pointcloud& pc);

// };

void find_clusters(Pointcloud& cloud);
void find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

std::vector<uint8_t> serializePC(Pointcloud& pc);
std::vector<uint8_t> serializePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);