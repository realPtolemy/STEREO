#include <thread>
#include <iostream>
#include <string>
#include "mapper/mapper.hpp"
#include "tracker/tracker.hpp"
#include "shared_state.hpp"
// #include <glog/logging.h>
#include <unistd.h>


int main(int argc, char **argv) {
    // std::cout << argv[0] << std::endl;
    // google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::INFO, "log/");

    // {
    //     std::cout << "SharedState constructed" << std::endl;
    //     LOG(INFO) << "Shared state construcotr";}

    // LOG(INFO) << "Starting";

    const char* newDir = "/home/fredrik/KEX/STEREO/";

    if (chdir(newDir) != 0) {
        perror("chdir failed");
        return 1;
    }
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    std::cout << "Current working directory: " << cwd << std::endl;

    SharedState shared_state;
    // LOG(INFO) << "Shared state created";
    Mapper mapper(shared_state);
    Tracker tracker(shared_state);
    std::thread mapper_thread(&Mapper::mapperRun, &mapper);
    std::thread tracker_thread(&Tracker::trackerRun, &tracker);

    mapper_thread.join();
    tracker_thread.join();
    return 0;
}