#include "robotname_hardware/udp.hpp"
#include <unistd.h>

int UDP::init() {

    int ret;

    socketID = socket(AF_INET, SOCK_DGRAM, 0);
    if(socketID < 0) {
        std::cout << "UDP Socket failed" << std::endl;
        return 0;
    }
    else {
        std::cout << "UDP Socket ID : " << socketID << std::endl;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip_server.c_str());

    ret = bind(socketID, (struct sockaddr*)&server_addr, sizeof(struct sockaddr_in));

    if(ret < 0){
        std::cout << "UDP Bind failed. " << errno << " : " << strerror(errno) << std::endl;
        close(socketID);
        return 0;
    }

    timeval t;
    t.tv_sec = 0;
    t.tv_usec = 1;

    ret = setsockopt(socketID, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));

    if(ret < 0) {
        std::cout << "UDP Set timeout failed" << std::endl;
        return 0;
    }

    std::cout << "UDP Start" << std::endl;

    return 1;
}

int UDP::setClient(std::string ip, uint16_t port) {
    ip_client = ip;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    client_addr.sin_addr.s_addr = inet_addr(ip.c_str());
}

int UDP::send(uint8_t* buff, size_t l) {
    socklen_t len = sizeof(client_addr);
    int ret = sendto(socketID, buff, l, MSG_CONFIRM, (sockaddr*)&client_addr, len);
    return ret;
}

int UDP::receive() {
    socklen_t len = 0;
    // memset(rx_buff, 0, 1024);
    int ret = recvfrom(socketID, rx_buff, 100, 0, (sockaddr*)& client_addr, &len);
    if(ret >= 0) rx_buff[ret] = (uint8_t)'/0';
    return ret;
}

UDP::~UDP() {
    close(socketID);
    std::cout << "UDP Socket closed" << std::endl;
}