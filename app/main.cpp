#include <thread>
#include <iostream>
#include "mapper/mapper.hpp"

int main(int argc, char **argv) {
    /*
        A thread for the mapper and tracker modules
        First run the mapper, so that some data is parsed that the tracker then can use
    */
    std::thread mapper_thread(mapper);
    // std::thread tracker_thread();

    mapper_thread.join();
    // tracker_thread.join();
}
