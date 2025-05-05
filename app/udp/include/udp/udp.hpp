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

class UDP
{
protected:
    int sockfd;
    sockaddr_in addr;

public:
    UDP();
    ~UDP();
};

class Client : public UDP
{

public:
    Client(int port);

    void send_string(const std::string& message, const std::string& clientIP, int clientPort);
    void send_uint8_t(const std::vector<uint8_t>& message, const std::string& clientIP);
};

class Server : public UDP
{
private:
    static constexpr size_t BUFFER_SIZE = 1024;
    std::array<uint8_t, BUFFER_SIZE> buffer;
    socklen_t len = sizeof(addr);
public:

    Server(int port);
    std::string receive_string();
    ssize_t read();
    std::tuple<uint16_t*, ssize_t> receive_aestream();
};

#endif