#include "udp/udp.hpp"

UDP::UDP() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        perror("Socket creation failed");
}

UDP::~UDP() {
    close(sockfd);
}

Server::Server(int port) {
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr)) < 0)
        perror("Bind failed");
    std::cout << "Server connected" << std::endl;
}

void Server::receive_aestream() {
    std::cout << "Reciving data" << std::endl;
    ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr *)&addr, &len);
    if (n < 0) {
        perror("recvfrom failed");
    }
    std::cout  << "X coord:" << (buffer[0] & 0x7FFF)
               << ", Y coord:" << (buffer[1] & 0x7FFF)
                // << ", polarity:" << (buffer[1] & 0x8000)
               << ", timestamp:" <<buffer[2] << buffer[3] <<"\n";
}

std::string Server::receive() {
    std::cout << "Reciving data" << std::endl;
    ssize_t n = recvfrom(sockfd, char_buffer, sizeof(buffer) - 1, 0,
                         (struct sockaddr *)&addr, &len);
    if (n < 0) {
        perror("recvfrom failed");
        return "";
    }
    char_buffer[n] = '\0';
    return char_buffer;
}

void Client::send(const std::string& message, const std::string& clientIP, int clientPort) {
    sockaddr_in clientAddr{};
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(clientPort);
    inet_pton(AF_INET, clientIP.c_str(), &clientAddr.sin_addr);
    sendto(sockfd, message.c_str(), message.size(), 0,
           (const struct sockaddr *)&clientAddr, sizeof(clientAddr));
}

Client::Client(int port) {
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
}
