#ifndef ROBOT_HANDLER_HPP
#define ROBOT_HANDLER_HPP
#include <thread>
#include "udp/udp.hpp"
#include <mutex>
#include <condition_variable>

class RobotHandler{
private:
    typedef UNITREE_LEGGED_SDK::HighCmd HighCmd;
    typedef UNITREE_LEGGED_SDK::HighState HighState;
    HighCmd highCmd_;
    HighState highState_;
    static constexpr int sendPort_ = 8082;
    static constexpr int recivePort_ = 8090;
    std::string IP_ = "192.168.123.161";
    std::mutex mutex_;
    std::thread commThread_;
    std::condition_variable cv_;
    UDP udp;
    bool sendCommand = false;

    void mainLoop();

public:
    RobotHandler();
    ~RobotHandler();
    void setCommand();
};
#endif