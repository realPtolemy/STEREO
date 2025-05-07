#include "udp/udp.hpp"
#include <thread>

UDP::UDP(int localPort, int dstPort, const std::string& clientIP)
{
    init_socket();
    configureDestination(localPort,clientIP);
    configureReceiver(dstPort);
}

UDP::UDP(int dstPort)
{
    init_socket();
    configureReceiver(dstPort);
}

UDP::UDP(int localPort, const std::string& clientIP)
{
    init_socket();
    configureDestination(localPort, clientIP);
}

UDP::~UDP()
{
    close(sockfd);
}

void UDP::init_socket()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) perror("Socket creation failed");
}

void UDP::configureReceiver(int port)
{
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);
    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
        perror("Bind failed");
}

void UDP::configureDestination(int port, const std::string& clientIP)
{
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, clientIP.c_str(), &dest_addr.sin_addr);
}

void UDP::send_string(const std::string& message)
{
    sendto(
        sockfd,
        message.c_str(),
        message.size(),
        0,
        (const struct sockaddr *)&dest_addr,
        sizeof(dest_addr)
    );
}


void UDP::send_uint8_t(const std::vector<uint8_t>& message)
{
    const size_t MAX_UDP_PAYLOAD = 60000;
    size_t offset = 0;
    std::cout << message.size() << std::endl;
    while (offset < message.size()) {
        size_t chunk_size = std::min(MAX_UDP_PAYLOAD, message.size() - offset);
        const uint8_t* chunk_ptr = message.data() + offset;
        ssize_t sent = sendto(
            sockfd,
            chunk_ptr,
            chunk_size,
            0,
            (const struct sockaddr *)&dest_addr,
            sizeof(dest_addr)
        );

        if (sent < 0) {
            perror("Failed to send message");
            break;
        }
        offset += sent;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

}

void UDP::send_A1_high_command(UNITREE_LEGGED_SDK::HighCmd command)
{
    sendto(
        sockfd,
        reinterpret_cast<const char*>(&command),
        sizeof(UNITREE_LEGGED_SDK::HighCmd),
        0,
        (const struct sockaddr *)&dest_addr,
        sizeof(dest_addr)
    );
}

UNITREE_LEGGED_SDK::HighState UDP::receive_A1_high_state()
{
    ssize_t n = read();

    if (n < 0) {
        std::cerr << "recvfrom failed" << std::endl;
        return UNITREE_LEGGED_SDK::HighState();
    }

    if (n < sizeof(UNITREE_LEGGED_SDK::HighState)) {
        std::cerr << "Incomplete HighState packet received" << std::endl;
        return UNITREE_LEGGED_SDK::HighState();
    }

    UNITREE_LEGGED_SDK::HighState state;
    std::memcpy(&state, buffer.data(), sizeof(UNITREE_LEGGED_SDK::HighState));
    return state;
}

std::tuple<uint16_t*, ssize_t> UDP::receive_aestream()
{
    ssize_t n = read();
    auto data = reinterpret_cast<uint16_t*>(buffer.data());
    return {data,n/2};
}

ssize_t UDP::read(){
    ssize_t n = recvfrom(
        sockfd,
        buffer.data(),
        BUFFER_SIZE,
        0,
        (struct sockaddr *)&local_addr,
        &len
    );

    if (n < 0) {
        std::cerr << "recvfrom failed" << std::endl;
    }

    return n;
}

std::string UDP::receive_string()
{
    int n = read();
    if (n <= 0) return {};

    std::string res(buffer.begin(), buffer.begin() + n);
    return res;
}