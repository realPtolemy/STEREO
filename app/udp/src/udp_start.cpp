// // main.cpp
// #include "udp/udp.hpp"

// int udp_start() {
//     Server server(12345);
//     Client client(12345);

//     server.send("Hello from server!", "127.0.0.1", 12345);
//     std::string msg = client.receive();
//     std::cout << "Client received: " << msg << std::endl;
//     return 0;
// }