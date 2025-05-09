#include "depth_perception/robot_handler.hpp"

RobotHandler::RobotHandler() : udp(recivePort_, sendPort_, IP_)
{
    highCmd_ = {0};
    highCmd_.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    //highCmd_.forwardSpeed = 0.0f;
    //highCmd_.sideSpeed = 0.0f;
    //highCmd_.rotateSpeed = 0.0f;
    //highCmd_.bodyHeight = 0.0f;
    //highCmd_.yaw = 0.0f;
    //highCmd_.pitch = 0.0f;
    //highCmd_.roll = 0.0f;
    highCmd_.mode = 2;
    highState_  = {0};
    sendCommand = true;
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
        if(sendCommand){
            udp.send_A1_high_command(highCmd_);
        }

        // Condition variable? Någon mutex måste finnas här!
        std::lock_guard<std::mutex> lock(mutex_);
        highState_ = udp.receive_A1_high_state();
        uint16_t buttonBits = highState_.wirelessRemote[0] | (highState_.wirelessRemote[1] << 8);
        std::cout << "R1 pressed?     " << ((buttonBits >> 0) & 1) << std::endl;
        std::cout << "L1 pressed?     " << ((buttonBits >> 1) & 1) << std::endl;
        std::cout << "Start pressed?  " << ((buttonBits >> 2) & 1) << std::endl;
        std::cout << "Select pressed? " << ((buttonBits >> 3) & 1) << std::endl;
        std::cout << "R2 pressed?     " << ((buttonBits >> 4) & 1) << std::endl;
        std::cout << "L2 pressed?     " << ((buttonBits >> 5) & 1) << std::endl;
        std::cout << "F1 pressed?     " << ((buttonBits >> 6) & 1) << std::endl;
        std::cout << "F2 pressed?     " << ((buttonBits >> 7) & 1) << std::endl;

        std::cout << "A pressed?      " << ((buttonBits >> 8) & 1) << std::endl;
        std::cout << "B pressed?      " << ((buttonBits >> 9) & 1) << std::endl;
        std::cout << "X pressed?      " << ((buttonBits >> 10) & 1) << std::endl;
        std::cout << "Y pressed?      " << ((buttonBits >> 11) & 1) << std::endl;
        std::cout << "Up pressed?     " << ((buttonBits >> 12) & 1) << std::endl;
        std::cout << "Right pressed?  " << ((buttonBits >> 13) & 1) << std::endl;
        std::cout << "Down pressed?   " << ((buttonBits >> 14) & 1) << std::endl;
        std::cout << "Left pressed?   " << ((buttonBits >> 15) & 1) << std::endl;
        //std::cout << "Forwards speed" <<highState_.forwardSpeed << std::endl;
        //std::cout <<"crc:" << highState_.crc <<std::endl;
        //uint8_t* ptr = reinterpret_cast<uint8_t*>(&highState_);
        //for (size_t i = 212; i < sizeof(UNITREE_LEGGED_SDK::HighState); ++i) {
        //      printf("%02x ", ptr[i]);
        //}
        std::cout << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

}
