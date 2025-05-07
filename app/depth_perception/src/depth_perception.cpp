#include "depth_perception/depth_perception.hpp"

DepthPerception::DepthPerception() : robotHandler()
{
	depth_perception_thread = std::thread(&DepthPerception::loop, this);
}

DepthPerception::~DepthPerception(){
	if(depth_perception_thread.joinable())
		depth_perception_thread.join();
}

void DepthPerception::start(Pointcloud& cloud){
	// The cloud is copied. TODO: Fix this later
	{
		std::lock_guard<std::mutex> lock(mtx_);
		running_ = true;
		this->cloud = cloud;
	}
	cv_.notify_one();
}

void DepthPerception::loop(){
	std::unique_lock<std::mutex> lock(mtx_);
	while (true)
	{
		/**
		 * Cluster extraction
		 *
		 */

		cv_.wait(lock, [this]{ return running_;});
		// Cluster extraction is not 100% done yet
		find_clusters(cloud);
		// Do some thinking about the clusters, then decide wheter not to "setCommand"
		robotHandler.setCommand();

	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthPerception::find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
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
    //   std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::cout << cloud_filtered->size() << std::endl;
	tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
	  std::cout << cluster_indices.size() << std::endl;
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
    return nullptr;
}

void DepthPerception::find_clusters(Pointcloud& cloud){
  /**
   * Example of how it can be done using PCL:
   *      https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
   */

   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

   // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PointXYZI> vg;
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
   vg.setInputCloud (cloud);
   vg.setLeafSize (0.1f, 0.1f, 0.1f);
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
  //    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
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
//  std::cout << cluster_indices.size() << std::endl;
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

std::vector<uint8_t> DepthPerception::serializePC(Pointcloud& pc)
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

std::vector<uint8_t> DepthPerception::serializePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
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

// void pointCloudGenerator(Pointcloud& pc){
// 	int height = pc->height;
// 	int width = pc->width;
// 	std::random_device rd;
//   	std::mt19937 gen(rd());
//   	std::uniform_real_distribution<float> coord_dis(-10.0, 10.0); // range for random points
// 	std::uniform_real_distribution<float> intensity_dis(0.0f, 1.0f);    // For intensity
// 	for(int i = 0; i < width; i++){
// 		for(int j = 0; j < height; j++){
//             int index = j * width + i;
//             auto& point = pc->points[index];
//             point.x = coord_dis(gen);
//             point.y = coord_dis(gen);
//             point.z = coord_dis(gen);
//             point.intensity = intensity_dis(gen);
// 		}
// 	}
// }

// void pointCloudGenerator(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) {
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::normal_distribution<float> dist(0.0, 0.2);  // Small spread

//     int num_clusters = 20;
//     int points_per_cluster = 500;

//     pc->clear();
//     pc->points.reserve(num_clusters * points_per_cluster);

//     for (int c = 0; c < num_clusters; ++c) {
//         float cx = (float)(std::rand() % 20 - 10);  // Cluster center X
//         float cy = (float)(std::rand() % 20 - 10);  // Cluster center Y
//         float cz = (float)(std::rand() % 5);        // Cluster center Z

//         for (int i = 0; i < points_per_cluster; ++i) {
//             pcl::PointXYZ pt;
//             pt.x = cx + dist(gen);
//             pt.y = cy + dist(gen);
//             pt.z = cz + dist(gen);
//             pc->points.push_back(pt);
//         }
//     }

//     pc->width = pc->points.size();
//     pc->height = 1;
//     pc->is_dense = true;
// }