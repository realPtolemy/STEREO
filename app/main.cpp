#include <thread>
#include <iostream>
#include "mapper/mapper.hpp"
#include "tracker/tracker.hpp"
// #include "talking/coordinator.hpp"
// #include "udp/udp.hpp"
#include "shared_state.hpp"

int main(int argc, char **argv) {
    /**
     *  What tracker udpates:
     *      - cmd_msg, a string
     *      - msg_pose
     *      - image msg
     *
     *  What mapper updates:
     *      - point cloud, line 542 and 563 in ES-PTAM mapper.cpp
     *      - cv_ptr->toImageMsg(), which is of type cv_bridge::CvImagePtr.
     *      See line 505 for defintion of cv_bridge::CvImagePtr.
     *
     *  Other things that needs to be updated:
     *      - tf_->waitForTransform, this is only for ROS, what we have to do
     *      is to implement this. It checks if there is a transform between two frames
     *      and if not, it waits for it. It's a blocking function. Run this on a thread
     *      with a condition variable and a mutex.
     *  From this 2 main things are trasmitted, msg_pose and point cloud.
     */
    // Mapper mapper;
    // Coordinator coordinator;

    // Server server(12345);
    // Client client(12345);

    // client.send("Hello from server!", "172.28.144.41", 12345);
    // std::string msg = server.receive();
    // std::cout << "Client received: " << msg << std::endl;

    SharedState shared_state;
    Mapper mapper(shared_state);
    // Tracker tracker(SharedState);
    return 0;
}