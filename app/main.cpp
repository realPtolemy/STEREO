#include <iostream>
#include <string>
#include "mapper/mapper.hpp"
#include "tracker/tracker.hpp"
#include "app/shared_state.hpp"
#include <unistd.h>
#include <depth_perception/robot_handler.hpp>

int main(int argc, char **argv) {
    // UDP udp(3333, 3334, "127.0.0.1");
    // Server server(3334);
    // pcl::PCDReader reader;
    // Pointcloud cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // cloud->height = 480;
    // cloud->width = 640;
    // cloud->resize(640*480);

    // pointCloudGenerator(cloud);
    // reader.read("data/table_scene_lms400.pcd", *cloud);
    // std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
    // // find_clusters(cloud);
    // std::vector<uint8_t> res = serializePC(cloud);
    // while(true){
    //     pointCloudGenerator(cloud);
    //     cloud_f = find_clusters(cloud);
    //     std::vector<uint8_t> res = serializePC(cloud_f);
    //     udp.send_uint8_t(res);
    //     std::string status = udp.receive_string();
    //     if(status != "Done"){
    //         std::cerr << "Package did not land well, message: " << status << std::endl;
    //         break;
    //     }
    //     std::cout << "Packet aknowledged!" << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(3));
    //     break;
    // }
    RobotHandler robotHandler;
    // SharedState shared_state;
    // Mapper mapper(shared_state);
    // Tracker tracker(shared_state);
    // std::thread mapper_thread(&Mapper::mapperRun, &mapper);
    // std::thread tracker_thread(&Tracker::trackerRun, &tracker);

    // mapper_thread.join();
    // tracker_thread.join();
    return 0;
}