#ifndef UDP_HPP
#define UDP_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#include <tuple>
#include <A1/comm.h>
#include <cstring>

class UDP
{
protected:
    int sockfd;
    sockaddr_in local_addr;
    sockaddr_in dest_addr;

    static constexpr size_t BUFFER_SIZE = 1024;
    std::array<uint8_t, BUFFER_SIZE> buffer;
    socklen_t len = sizeof(local_addr);

public:
    UDP(int localPort, int dstPort, const std::string& clientIP);
    UDP(int destintaionPort, const std::string& clientIP);
    UDP(int localPort);
    void init_socket();
    void configureReceiver(int port);
    void configureDestination(int port, const std::string& clientIP);
    ~UDP();

    // Receive data
    std::string receive_string();
    ssize_t read();
    std::tuple<uint16_t*, ssize_t> receive_aestream();
    UNITREE_LEGGED_SDK::HighState receive_A1_high_state();

    // Send data
    void send_string(const std::string& message);
    void send_uint8_t(const std::vector<uint8_t>& message);
    void send_A1_high_command(UNITREE_LEGGED_SDK::HighCmd command);
};
#endif