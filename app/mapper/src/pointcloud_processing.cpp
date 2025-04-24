#include "mapper/pointcloud_processing.hpp"


// PointcloudProcessing::PointcloudProcessing(int port) : client(port) {}

// PointcloudProcessing::PointcloudProcessing() : client(0) {}

void find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    /**
     * Example of how it can be done using PCL:
     *      https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
     */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : cluster.indices) {
        cloud_cluster->push_back((*cloud_filtered)[idx]);
      } //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << j;
      writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
      j++;
    }
}

void find_clusters(Pointcloud& cloud){
    /**
     * Example of how it can be done using PCL:
     *      https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
     */

     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZI> vg;
     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
     vg.setInputCloud (cloud);
     vg.setLeafSize (0.01f, 0.01f, 0.01f);
     vg.filter (*cloud_filtered);
     std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

     // Create the segmentation object for the planar model and set all the parameters
     pcl::SACSegmentation<pcl::PointXYZI> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
     pcl::PCDWriter writer;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100);
     seg.setDistanceThreshold (0.02);

     int nr_points = (int) cloud_filtered->size ();
     while (cloud_filtered->size () > 0.3 * nr_points)
     {
       // Segment the largest planar component from the remaining cloud
       seg.setInputCloud (cloud_filtered);
       seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0)
       {
         std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
         break;
       }
       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZI> extract;
       extract.setInputCloud (cloud_filtered);
       extract.setIndices (inliers);
       extract.setNegative (false);
       // Get the points associated with the planar surface
       extract.filter (*cloud_plane);
       std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
       // Remove the planar inliers, extract the rest
       extract.setNegative (true);
       extract.filter (*cloud_f);
       *cloud_filtered = *cloud_f;
     }
     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
     tree->setInputCloud (cloud_filtered);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
     ec.setClusterTolerance (0.02); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);

     int j = 0;
     for (const auto& cluster : cluster_indices)
     {
       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
       for (const auto& idx : cluster.indices) {
         cloud_cluster->push_back((*cloud_filtered)[idx]);
       } //*
       cloud_cluster->width = cloud_cluster->size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;

       std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
       std::stringstream ss;
       ss << std::setw(4) << std::setfill('0') << j;
       writer.write<pcl::PointXYZI> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
       j++;
     }
}

std::vector<uint8_t> serializePC(Pointcloud& pc)
{
    pcl::PCLPointCloud2 blob;
    pcl::toPCLPointCloud2(*pc, blob);
    auto& raw = blob.data;

    int maxCSize = LZ4_compressBound(raw.size());
    std::vector<char> compressed(maxCSize);
    int cSize = LZ4_compress_default(
      reinterpret_cast<const char*>(raw.data()),
      compressed.data(),
      raw.size(),
      maxCSize
    );

    if (cSize <= 0) {
        return std::vector<uint8_t>();
    }
    compressed.resize(cSize);

    std::vector<uint8_t> packet(4 + cSize);
    uint32_t uncompressed_size = static_cast<uint32_t>(raw.size());
    std::memcpy(packet.data(), &uncompressed_size, 4);
    std::memcpy(packet.data() + 4, compressed.data(), cSize);

    return packet;
    // sendPC();

}

std::vector<uint8_t> serializePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
    pcl::PCLPointCloud2 blob;
    pcl::toPCLPointCloud2(*pc, blob);
    auto& raw = blob.data;

    int maxCSize = LZ4_compressBound(raw.size());
    std::vector<char> compressed(maxCSize);
    int cSize = LZ4_compress_default(
      reinterpret_cast<const char*>(raw.data()),
      compressed.data(),
      raw.size(),
      maxCSize
    );

    if (cSize <= 0) {
        return std::vector<uint8_t>();
    }
    compressed.resize(cSize);

    std::vector<uint8_t> packet(8 + cSize);
    uint32_t uncompressed_size  = static_cast<uint32_t>(raw.size());
    uint32_t compressed_size = static_cast<uint32_t>(cSize);
    std::cout << "compressed size: " << compressed_size << std::endl;
    std::cout << "compressed size: " << uncompressed_size << std::endl;
    // std::memcpy(packet.data(), &uncompressed_size, 4);
    std::memcpy(packet.data() + 0, &compressed_size, sizeof(compressed_size));
    std::memcpy(packet.data() + 4, &uncompressed_size,  sizeof(uncompressed_size));
    std::memcpy(packet.data() + 8, compressed.data(), cSize);

    return packet;
}

// void PointcloudProcessing::sendPC()
// {
//     /**
//      * Send the point cloud on a TCP or UDP client(look more into what's best here)
//      * A server has to be initilzed somehere, not here because then the server has to be
//      * started up over and over again!
//      */
// }