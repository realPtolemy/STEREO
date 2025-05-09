#include "depth_perception/robot_handler.hpp"

RobotHandler::RobotHandler() : udp(recivePort_, sendPort_, IP_)
{
    highCmd_ = {0};
    highCmd_.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    highCmd_.forwardSpeed = 0;
    highCmd_.mode = 0;
    highState_  = {0};
    sendCommand = false;
    commThread_ = std::thread(&RobotHandler::mainLoop, this);
}

RobotHandler::~RobotHandler()
{
    if(commThread_.joinable())
        commThread_.join();
}

void RobotHandler::setCommand()
{
    std::lock_guard<std::mutex> lock(mutex_);
    sendCommand = true;
}

void RobotHandler::mainLoop()
{
    while (true)
    {
        // Condition variable? Någon mutex måste finnas här!
        highState_ = udp.receive_A1_high_state();
        if(sendCommand){
            udp.send_A1_high_command(highCmd_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

}
