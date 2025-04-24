#include "udp/udp.hpp"
#include <thread>

UDP::UDP()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        perror("Socket creation failed");
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;

}

UDP::~UDP()
{
    close(sockfd);
}

void Client::send_string(const std::string& message, const std::string& clientIP, int clientPort)
{
    // sockaddr_in clientAddr{};
    // clientAddr.sin_family = AF_INET;
    // clientAddr.sin_port = htons(clientPort);
    inet_pton(AF_INET, clientIP.c_str(), &addr.sin_addr);
    sendto(sockfd, message.c_str(), message.size(), 0,
           (const struct sockaddr *)&addr, sizeof(addr));
}


void Client::send_uint8_t(const std::vector<uint8_t>& message, const std::string& clientIP)
{
    const size_t MAX_UDP_PAYLOAD = 60000;
    size_t offset = 0;
    inet_pton(AF_INET, clientIP.c_str(), &addr.sin_addr);
    std::cout << message.size() << std::endl;
    while (offset < message.size()) {
        size_t chunk_size = std::min(MAX_UDP_PAYLOAD, message.size() - offset);
        const uint8_t* chunk_ptr = message.data() + offset;
        ssize_t sent = sendto(
            sockfd,
            chunk_ptr,
            chunk_size,
            0,
            (const struct sockaddr *)&addr,
            sizeof(addr)
        );

        if (sent < 0) {
            perror("Failed to send message");
            break;
        }
        offset += sent;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

}

Client::Client(int port)
{
    addr.sin_port = htons(port);
}


Server::Server(int port)
{

    addr.sin_port = htons(port);
    if (bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr)) < 0)
        perror("Bind failed");
    std::cout << "Server connected" << std::endl;
}

void Server::receive_aestream()
{
    std::cout << "Reciving data" << std::endl;
    read();
    auto data = reinterpret_cast<uint16_t*>(buffer.data());
    // std::cout  << "X coord:" << (buffer[0] & 0x7FFF)
    //            << ", Y coord:" << (buffer[1] & 0x7FFF)
    //             // << ", polarity:" << (buffer[1] & 0x8000)
    //            << ", timestamp:" <<buffer[2] << buffer[3] <<"\n";
}
ssize_t Server::read(){
    std::cout << "Reciving data" << std::endl;
    ssize_t n = recvfrom(
        sockfd,
        buffer.data(),
        BUFFER_SIZE,
        0,
        (struct sockaddr *)&addr,
        &len
    );

    if (n < 0) {
        std::cerr << "recvfrom failed" << std::endl;
    }

    return n;
}

std::string Server::receive_string()
{
    int n = read();
    if (n <= 0) return {};

    std::string res(buffer.begin(), buffer.begin() + n);
    return res;
}