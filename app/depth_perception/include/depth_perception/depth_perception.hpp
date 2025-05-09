#ifndef DEPTH_PERCEPTION_HPP
#define DEPTH_PERCEPTION_HPP

/**
 * Hanterar hela vår pipeline, alltså cluster
 * extraction -> cluster processing -> robot command
 *
 */
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
#include <iomanip>
#include <lz4.h>
#include "depth_perception/robot_handler.hpp"

class DepthPerception{
private:
    RobotHandler robotHandler;
    void find_clusters(Pointcloud& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<uint8_t> serializePC(Pointcloud& pc);
    std::vector<uint8_t> serializePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);
    // void pointCloudGenerator(Pointcloud& pc);
    // void pointCloudGenerator(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);

    void loop();
    std::thread depth_perception_thread;
    bool running_ = false;
    std::condition_variable cv_;
    std::mutex mtx_;
    Pointcloud cloud;
public:
    DepthPerception();
    ~DepthPerception();
    void start(Pointcloud& cloud);
};
#endif