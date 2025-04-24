#include <iostream>
#include <string>
#include "mapper/mapper.hpp"
#include "tracker/tracker.hpp"
#include "app/shared_state.hpp"
#include "mapper/pointcloud_processing.hpp"
#include <unistd.h>

int main(int argc, char **argv) {
    // const char* newDir = "/home/fredrik/KEX/STEREO/";

    // if (chdir(newDir) != 0) {
    //     perror("chdir failed");
    //     return 1;
    // }
    // char cwd[1024];
    // getcwd(cwd, sizeof(cwd));
    // std::cout << "Current working directory: " << cwd << std::endl;
    Client client(3333);
    Server server(3334);
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("data/table_scene_lms400.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
    // find_clusters(cloud);
    std::vector<uint8_t> res = serializePC(cloud);
    while(true){
        client.send_uint8_t(res, "127.0.0.1");
        std::string status = server.receive_string();
        if(status != "Done"){
            std::cerr << "Package did not land well, message: " << status << std::endl;
            break;
        }
        std::cout << "Packet aknowledged!" << std::endl;
        break;
    }

    // SharedState shared_state;
    // Mapper mapper(shared_state);
    // Tracker tracker(shared_state);
    // std::thread mapper_thread(&Mapper::mapperRun, &mapper);
    // std::thread tracker_thread(&Tracker::trackerRun, &tracker);

    // mapper_thread.join();
    // tracker_thread.join();
    return 0;
}