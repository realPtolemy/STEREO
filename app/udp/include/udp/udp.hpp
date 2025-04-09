#include <sys/socket.h>
#include <netinet/in.h>  // For sockaddr_in
#include <arpa/inet.h>   // For inet_addr
#include <unistd.h>      // For close()
#include <string.h>      // For memset()
#include <iostream>      // For logging/debug
#include <vector>

class UDP{
protected:
    int sockfd;
    sockaddr_in addr;
public:
    UDP();
    virtual ~UDP();
};

class Client : public UDP{

public:
    Client(int port);

    void send(const std::string& message, const std::string& clientIP, int clientPort);
};

class Server : public UDP{
private:
    char char_buffer[1024];
    uint16_t buffer[1024];
    socklen_t len = sizeof(addr);
public:
    Server(int port);
    std::string receive();
    void receive_aestream();
    void buildPacket_pcl();
    void buildPacket_cvIMG();
};