#include "mapper/pointcloud_processing.hpp"

void find_clusters(Pointcloud& pc){
    /**
     * Example of how it can be done using PCL:
     *      https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
     */
}

void serializePC(Pointcloud& pc){
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
        // ROS_ERROR("LZ4 compression failed");
        return;
    }
    compressed.resize(cSize);

    // 4) Frame it (prepend uncompressed size as 4 bytes)
    std::vector<uint8_t> packet(4 + cSize);
    uint32_t uncompressed_size = static_cast<uint32_t>(raw.size());
    std::memcpy(packet.data(), &uncompressed_size, 4);
    std::memcpy(packet.data() + 4, compressed.data(), cSize);

    // 5) Send via ROS topic (e.g. as a std_msgs::ByteMultiArray)
    // std_msgs::ByteMultiArray msg;
    // msg.data = std::move(packet);

    sendPC();

}

void sendPC(){
    /**
     * Send the point cloud on a TCP or UDP client(look more into what's best here)
     * A server has to be initilzed somehere, not here because then the server has to be
     * started up over and over again!
     */
}