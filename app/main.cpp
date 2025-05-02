#include <iostream>
#include <string>
#include "mapper/mapper.hpp"
#include "tracker/tracker.hpp"
#include "app/shared_state.hpp"
#include "mapper/pointcloud_processing.hpp"
#include <unistd.h>

int main(int argc, char **argv) {
    std::cout << "[main] Program starting..." << std::endl;

    // Set working directory
    /*const char* newDir = "/home/fredrik/KEX/STEREO/";
    if (chdir(newDir) != 0) {
        perror("[main] chdir failed");
        return 1;
    }
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "[main] Current working directory: " << cwd << std::endl;
    } else {
        std::cerr << "[main] Failed to get current working directory" << std::endl;
    }
    */

    // Point cloud processing (unchanged)
    /* 
    Client client(3333);
    Server server(3334);
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("data/table_scene_lms400.pcd", *cloud);
    std::cout << "[main] PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;
    std::vector<uint8_t> res = serializePC(cloud);
    */ 

    // Initialize and run Mapper and Tracker
    std::cout << "[main] Initializing SharedState, Mapper, and Tracker..." << std::endl;
    SharedState shared_state;
    Mapper mapper(shared_state);
    Tracker tracker(shared_state);
    
    std::cout << "[main] Starting mapper and tracker threads..." << std::endl;
    std::thread mapper_thread(&Mapper::mapperRun, &mapper);
    std::thread tracker_thread(&Tracker::trackerRun, &tracker);

    std::cout << "[main] Waiting for mapper and tracker threads to complete..." << std::endl;
    mapper_thread.join();
    tracker_thread.join();
    
    std::cout << "[main] Program completed." << std::endl;
    return 0;
}